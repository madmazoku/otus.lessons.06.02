
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

    long width = display_mode.w >> 1;
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

    SDL_Surface *scr = SDL_GetWindowSurface(win);
    SDL_Surface *img = SDL_CreateRGBSurface(0, scr->w, scr->h, 32, 0, 0, 0, 0);

    auto start = std::chrono::system_clock::now();
    size_t count = 0;
    auto last = start;
    size_t last_count = count;
    double time_step = 0;
    bool run = true;
    while (run) {
        auto loop_start = std::chrono::system_clock::now();
        ++count;

        SDL_LockSurface(img);

        uint8_t* py = (uint8_t*)(img->pixels);
        SDL_memset(py, 0, img->h * img->pitch);
        for(size_t y = 0; y < img->h; ++y, py += img->pitch) {
            uint32_t* px = (uint32_t*)py;
            for(size_t x = 0; x < img->w; ++x, ++px) {
                *px = SDL_MapRGBA(img->format, (x+count) & 0xff, (y+count) & 0xff, (x+y+count) & 0xff, 0x00);
            }
        }

        SDL_UnlockSurface(img);

        SDL_BlitSurface(img, nullptr, scr, nullptr);

        SDL_UpdateWindowSurface(win);

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

            console->info("[{0} / {1}] fps: {2}; time_step: {3}", full_elapsed.count(), count, fps, time_step);

            last = end;
            last_count = count;
        }
    }

    SDL_FreeSurface(img);
    SDL_FreeSurface(scr);

    SDL_DestroyWindow(win);
    SDL_Quit();
}