    
#include "../bin/version.h"

#include <iostream>
#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

#include <SDL2/SDL.h>
#pragma comment(lib, "SDL2.lib")
#pragma comment(lib, "SDL2main.lib")

#include "solve.h"

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

    size_t cell_count = width-10;
    double gam = 0.015;
    double guk = 100;
    double pond = 1e-5;
    std::vector<double> state(2 * cell_count);
    for(auto &s : state)
        s = 0;
    double s_max = state[0];
    double s_min = s_max;
    double v_max = state[1];
    double v_min = v_max;

    auto lambda = [cell_count, gam, guk, pond](const decltype(state) &s , decltype(state) &dsdt , const double t) 
    {
        for(size_t n = 1; n < cell_count - 1; ++n) {
            dsdt[n * 2] = s[n * 2 + 1];
            dsdt[n * 2 + 1] = -gam * s[n * 2 + 1] + guk * ((s[(n - 1) * 2] + s[(n + 1) * 2]) - 2*s[n * 2]) - pond;
        }
        size_t left_n = 0;
        size_t right_n = cell_count - 1;
        double left_s = 0;
        double right_s = 0;
        dsdt[left_n * 2] = s[left_n * 2 + 1];
        dsdt[left_n * 2 + 1] = -gam * s[left_n * 2 + 1] + guk * ((left_s + s[(left_n + 1) * 2]) - 2*s[left_n * 2]) - pond;
        dsdt[right_n * 2] = s[right_n * 2 + 1];
        dsdt[right_n * 2 + 1] = -gam * s[right_n * 2 + 1] + guk * ((s[(right_n - 1) * 2] + right_s) - 2*s[right_n * 2]) - pond;
    };

    SDL_Point* points = new SDL_Point[cell_count];

    auto start = std::chrono::system_clock::now();
    size_t count = 0;
    auto last = start;
    size_t last_count = count;
    double t = 0;
    double time_step = 1.0 / 60.0;
    bool run = true;
    while (run) {
        auto loop_start = std::chrono::system_clock::now();
        ++count;

        size_t steps = boost::numeric::odeint::integrate(lambda, state, t, t + 0.025 , time_step);
        t += 0.025;

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0xff);
        SDL_RenderClear(renderer);

        SDL_Rect rect;
        rect.x = 5;
        rect.y = 5;
        rect.w = width - 10;
        rect.h = height - 10;
        SDL_SetRenderDrawColor(renderer, 0x7f, 0x7f, 0x7f, 0xff);
        SDL_RenderDrawRect(renderer, &rect);

        for(size_t n = 0; n < cell_count; ++n) {
            if(s_max < state[n * 2])
                s_max = state[n * 2];
            if(v_max < state[n * 2 + 1])
                v_max = state[n * 2 + 1];
            if(s_min > state[n * 2])
                s_min = state[n * 2];
            if(v_min > state[n * 2 + 1])
                v_min = state[n * 2 + 1];
        }
        if(s_max * s_min < 0) {
            size_t y = s_max / (s_max - s_min) * (height - 10) + 5;
            SDL_RenderDrawLine(renderer, 5, y, width-5, y);
        }

        {
            for(size_t n = 0; n < cell_count; ++n) {
                points[n].x = n + 5;
                points[n].y = (s_max - state[n*2]) / (s_max - s_min) * (height - 10) + 5;
            }
            SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);
            SDL_RenderDrawLines(renderer, points, cell_count);
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
                full_elapsed.count(), count, fps, time_step, t, v_max, v_min);

            last = end;
            last_count = count;
        }
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(win);
    SDL_Quit();
}