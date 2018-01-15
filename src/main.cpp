    
#include "../bin/version.h"

#include <iostream>
#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

#include <SDL2/SDL.h>
#pragma comment(lib, "SDL2.lib")
#pragma comment(lib, "SDL2main.lib")

#include "solve.h"

double sqr(double x) { return x*x; }

void main_body();

int main(int argc, char** argv) 
{
    auto console = spdlog::stdout_logger_st("console");
    console->info("Wellcome!");

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
    ("help,h", "print usage message")
    ("version,v", "print version number");

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
    } else if (vm.count("version")) {
        std::cout << "Build version: " << build_version() << std::endl;
        std::cout << "Boost version: " << (BOOST_VERSION / 100000) << '.' << (BOOST_VERSION / 100 % 1000) << '.' << (BOOST_VERSION % 100) << std::endl;
    } else {
        main_body();
    }

    console->info("Goodby!");

    return 0;
}

double sgn(double x) { return x < 0 ? -1 : x > 0 ? 1 : 0; }

void main_body()
{
    auto console = spdlog::get("console");

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        console->error("SDL_Init Error: {1}", SDL_GetError());
        throw std::runtime_error("SDL_Init");
    }

    SDL_DisplayMode display_mode;
    if (SDL_GetCurrentDisplayMode(0, &display_mode) != 0) {
        console->error("SDL_GetCurrentDisplayMode Error: {1}", SDL_GetError());
        SDL_Quit();
        throw std::runtime_error("SDL_GetCurrentDisplayMode");
    }

    long width = display_mode.w - 100;// >> 1;
    long height = display_mode.h >> 1;
    width = height;

    SDL_Window *win = SDL_CreateWindow(
                          "Hellow World!",
                          SDL_WINDOWPOS_CENTERED,
                          SDL_WINDOWPOS_CENTERED,
                          width,
                          height,
                          SDL_WINDOW_SHOWN
                      );
    if (win == nullptr) {
        console->error("SDL_CreateWindow Error: {1}", SDL_GetError());
        SDL_Quit();
        throw std::runtime_error("SDL_CreateWindow");
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED); 
    if (win == nullptr) {
        console->error("SDL_CreateRenderer Error: {1}", SDL_GetError());
        SDL_Quit();
        throw std::runtime_error("SDL_CreateRenderer");
    }

    // d^2*u/dt^2 = c^2 * (d^2*u / dx^2 ) - mu * du / dt - pond

    double mu = 0.00015;
    double c = 10.0;
    double p = 0; //1e-5;
    double dt = 0.1;
    double dh = 0.1;
    double t = 0;

    size_t zoom = 5;
    size_t zoom_t = 500;
    double dhz = dh / zoom;
    double dtz = dt / zoom_t;

    double c2 = sqr(c * dtz / dhz);
    double mu_dt = mu * dtz;
    double dt2 = sqr(dtz);

    double* state_prev = new double[width*zoom];
    double* state_curr = new double[width*zoom];
    double* state_next = new double[width*zoom];

    memset(state_prev, 0, sizeof(double)*width*zoom);
    memset(state_curr, 0, sizeof(double)*width*zoom);
    memset(state_next, 0, sizeof(double)*width*zoom);

    double s_max = 1.0;
    double s_min = -1.0;

    SDL_Point* points = new SDL_Point[width*zoom];

    auto start = std::chrono::system_clock::now();
    double time_step = 0;
    size_t count = 0;
    auto last = start;
    size_t last_count = count;
    bool run = true;
    bool calc = true;
    while (run) {
        auto loop_start = std::chrono::system_clock::now();
        ++count;

        if(calc) {
            for(size_t z = 0; z < zoom_t; ++z) {
                for(size_t x = 1; x < width*zoom - 1; ++x) {
                    double s = c2 * (state_curr[x - 1] + state_curr[x + 1] - 2 * state_curr[x]);
                    state_next[x] = (2 - mu_dt) * state_curr[x] - (1 - mu_dt) * state_prev[x] - p * dt2 + s;
                }

                std::swap(state_prev, state_curr);
                std::swap(state_curr, state_next);
                t += dtz;
            }
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0xff);
        SDL_RenderClear(renderer);

        for(size_t n = 1; n < width*zoom; ++n) {
            if(s_max < state_curr[n])
                s_max = state_curr[n];
            if(s_min > state_curr[n])
                s_min = state_curr[n];
        }
        if(s_max * s_min < 0) {
            size_t y = s_max / (s_max - s_min) * height;
            SDL_RenderDrawLine(renderer, 0, y, width, y);
        }

        if(s_min != s_max) {
            for(size_t n = 0; n < width * zoom; ++n) {
                points[n].x = n / zoom;
                points[n].y = (s_max - state_curr[n]) / (s_max - s_min) * height;
            }
            SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);
            SDL_RenderDrawLines(renderer, points, width*zoom);
        }

        SDL_RenderPresent(renderer);

        // SDL_UpdateWindowSurface(win);

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT
                || event.type == SDL_KEYDOWN
                || event.type == SDL_KEYUP) {
                run = false;
            }
            if(event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT && calc) {
                double bx = event.button.x * zoom;
                for(size_t n = 1; n < width*zoom - 1; ++n) {
                    double d = bx < n ? (n - bx)*(-dhz) : (bx - n)*dhz;
                    double e = exp(-sqr(d));
                    state_prev[n] -= e;
                    state_curr[n] -= e;

                }
                calc = false;
            }
            if(event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT) {
                calc = true;
            }
        }

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> full_elapsed = end - start;
        std::chrono::duration<double> last_elapsed = end - last;
        std::chrono::duration<double> loop_elapsed = end - loop_start;
        time_step = loop_elapsed.count();

        if (!run || last_elapsed.count() >= 1) {
            int frames = count - last_count;
            double fps = ((double)frames) / last_elapsed.count();

            SDL_SetWindowTitle(win, ("Hello World! FPS: " + boost::lexical_cast<std::string>(fps)).c_str());

            console->info("[{0} / {1}] fps: {2}; time_step: {3}; time: {4}; max: {5}; min: {6}", 
                full_elapsed.count(), count, fps, time_step, t, s_max, s_min);

            last = end;
            last_count = count;
        }
    }

    delete[] state_next;
    delete[] state_curr;
    delete[] state_prev;

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(win);
    SDL_Quit();
}