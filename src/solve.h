#pragma once

#include <boost/numeric/odeint.hpp>
#include <chrono>
#include <thread>

// harmonic oscillator with friction d^2x/dt^2 = -x + gam * dx/dt
// q = x
// p = dx/dt = dq/dt
// w = (q, p)
// dw/dt = (dq/dt, dp/dt) = (p, -q + gam * p)
// (dx/dt, d^2x/dt^2)

typedef std::vector<double> state_type;

constexpr double gam = 0.25;
constexpr double t_step = 0.2;

// dx/dt = f(x)
void harmonic_oscillator( const state_type &x , state_type &dxdt , const double t )
{
    dxdt[0] = x[1];
    dxdt[1] = -x[0] - gam*x[1] + sin(t) * 0.15;
}

void harmonic_solve()
{
    state_type x(2);
    x[0] = 1.0;
    x[1] = 0.0;

    for(double t = 0.0; t < 10.0; t += t_step) {
// runge_kutta54_cash_karp
// update x and return number of steps
        state_type px = x;
        size_t steps = boost::numeric::odeint::integrate(
            harmonic_oscillator,    // rhs
            x,                      // initial state
            t,                      // start time
            t+t_step,               // end time
            t_step                  // initial time step
        );
        size_t pxs = (size_t)((px[0] + 1.0) * 40 + 0.5);
        size_t xs = (size_t)((x[0] + 1.0) * 40 + 0.5);
        size_t offset, length;
        if(pxs < xs) {
            offset = pxs;
            length = xs - pxs;
        } else {
            offset = xs;
            length = pxs - xs;
        }
        std::cout << "[ " << std::setw(7) << std::setprecision(4) << std::fixed << (t + t_step) << " ] ";
        std::cout << std::setw(7) << std::setprecision(4) << std::fixed << px[0] << " -> ";
        std::cout << std::setw(7) << std::setprecision(4) << std::fixed << x[0] << ": ";
        std::cout << std::resetiosflags(std::ios_base::floatfield) << steps << " ";
        std::cout << std::string(offset, ' ') << std::string(length > 0 ? length : 1, '*');
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}