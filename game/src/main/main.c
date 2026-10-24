#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio_device.h"
#include "city_list.h"
#include "city_menu.h"
#include "driving_stage.h"
#include "log.h"
#include "renderer.h"
#include "window.h"

typedef struct {
  window_options_t window;
  stage_options_t stage;
  bool help;
  bool no_audio;
  bool scene_chosen;   /* on the command line, so no menu */
  char city_path[256]; /* the package for --city=NAME */
} options_t;

#define CITIES_DIRECTORY "assets_cooked/cities"

static void print_help(void) {
  printf(
      "gta-like: drive a car on a test track or in a generated city.\n\n"
      "  --scene=track            the test track (default)\n"
      "  --scene=city:<package>   a cooked city package directory\n"
      "  --city=<name>            a cooked city by name, such as pordenone or "
      "barletta\n"
      "                           (with no scene given, a menu offers them)\n"
      "  --vehicle=<file>         vehicle parameters "
      "(assets/vehicles/hatchback.cfg)\n"
      "  --width=N --height=N     window size (1280x720)\n"
      "  --fullscreen             borderless fullscreen\n"
      "  --no-vsync               do not wait for the display\n"
      "  --no-audio               do not open an audio device\n"
      "  --mute                   open audio but start muted\n"
      "  --time=H                 start at hour H (0-24), default 10\n"
      "  --traffic=N              moving cars in the city (default 120, 0 for "
      "none)\n"
      "  --pedestrians=N          people on the pavements (default 250)\n"
      "  --rain                   start in the rain (T toggles it)\n"
      "  --hour-seconds=S         real seconds per game hour, 0 freezes time\n"
      "  --debug                  start with the debug overlay on\n"
      "  --help\n\n"
      "Controls: arrows or WASD drive, space handbrake, R reset, P pause,\n"
      "C view (chase, bonnet, cockpit), V next car, F get out or in (walk with "
      "the\n"
      "arrows, space to run, G shoots, Q punches), H horn, F1 debug, M map,\n"
      "T rain, N mute, F11 fullscreen, Esc quit. A game "
      "controller\n"
      "steers with the left stick, triggers for throttle and brake, A "
      "handbrake,\n"
      "Y reset, Start pause.\n");
}

static bool parse_options(int argc, char* argv[], options_t* options) {
  memset(options, 0, sizeof(*options));
  options->window = window_default_options("gta-like");
  options->stage.scene = SCENE_TEST_TRACK;
  options->stage.start_hour = 10.0f;
  options->stage.hour_seconds = 30.0f;
  options->stage.traffic_cars = 120;
  options->stage.pedestrians = 250;
  options->stage.rain = 0.0f;
  for (int i = 1; i < argc; i++) {
    const char* argument = argv[i];
    if (strcmp(argument, "--help") == 0) {
      options->help = true;
    } else if (strcmp(argument, "--fullscreen") == 0) {
      options->window.fullscreen = true;
    } else if (strcmp(argument, "--no-vsync") == 0) {
      options->window.vsync = false;
    } else if (strcmp(argument, "--no-audio") == 0) {
      options->no_audio = true;
    } else if (strcmp(argument, "--mute") == 0) {
      options->stage.muted = true;
    } else if (strcmp(argument, "--debug") == 0) {
      options->stage.show_debug = true;
    } else if (strncmp(argument, "--time=", 7) == 0) {
      options->stage.start_hour = (float)atof(argument + 7);
    } else if (strncmp(argument, "--hour-seconds=", 15) == 0) {
      options->stage.hour_seconds = (float)atof(argument + 15);
    } else if (strncmp(argument, "--traffic=", 10) == 0) {
      options->stage.traffic_cars = atoi(argument + 10);
    } else if (strncmp(argument, "--pedestrians=", 14) == 0) {
      options->stage.pedestrians = atoi(argument + 14);
    } else if (strcmp(argument, "--rain") == 0) {
      options->stage.rain = 1.0f;
    } else if (strncmp(argument, "--width=", 8) == 0) {
      options->window.width = atoi(argument + 8);
    } else if (strncmp(argument, "--height=", 9) == 0) {
      options->window.height = atoi(argument + 9);
    } else if (strncmp(argument, "--vehicle=", 10) == 0) {
      options->stage.vehicle_file = argument + 10;
    } else if (strcmp(argument, "--scene=track") == 0) {
      options->stage.scene = SCENE_TEST_TRACK;
      options->scene_chosen = true;
    } else if (strncmp(argument, "--scene=city:", 13) == 0) {
      options->stage.scene = SCENE_CITY;
      options->stage.city_package = argument + 13;
      options->scene_chosen = true;
    } else if (strncmp(argument, "--city=", 7) == 0) {
      snprintf(options->city_path, sizeof(options->city_path), "%s/%s",
               CITIES_DIRECTORY, argument + 7);
      options->stage.scene = SCENE_CITY;
      options->stage.city_package = options->city_path;
      options->scene_chosen = true;
    } else {
      fprintf(stderr, "unknown option %s\n", argument);
      return false;
    }
  }
  if (options->window.width < 320 || options->window.height < 200) {
    fprintf(stderr, "window size must be at least 320x200\n");
    return false;
  }
  return true;
}

int main(int argc, char* argv[]) {
  options_t options;
  if (!parse_options(argc, argv, &options)) {
    print_help();
    return EXIT_FAILURE;
  }
  if (options.help) {
    print_help();
    return EXIT_SUCCESS;
  }
  window_t window;
  if (!window_open(&window, options.window)) return EXIT_FAILURE;
  renderer_t* renderer =
      renderer_create(window.drawable_width, window.drawable_height);
  if (!renderer) {
    window_close(&window);
    return EXIT_FAILURE;
  }
  city_entry_t cities[CITY_LIST_MAX];
  size_t city_count = 0;
  if (!options.scene_chosen)
    city_count = city_list(CITIES_DIRECTORY, cities, CITY_LIST_MAX);
  options.stage.has_menu = city_count > 0;
  audio_device_t audio;
  bool has_audio = !options.no_audio && audio_device_open(&audio);
  int status = EXIT_SUCCESS;
  int choice = 0;
  bool again = true;
  while (again) {
    again = false;
    if (city_count > 0) {
      /* The start menu: the test track or a city; Backspace comes back. */
      choice = city_menu_run(&window, renderer, cities, city_count, choice + 1);
      if (choice == -2) break;
      options.stage.scene = choice >= 0 ? SCENE_CITY : SCENE_TEST_TRACK;
      if (choice >= 0) options.stage.city_package = cities[choice].path;
    }
    driving_stage_t stage;
    if (driving_stage_init(&stage, &window, renderer, has_audio ? &audio : NULL,
                           options.stage)) {
      driving_stage_run(&stage);
      again = stage.back_to_menu;
    } else {
      LOG_ERROR("main", "could not start the driving stage");
      status = EXIT_FAILURE;
    }
    driving_stage_destroy(&stage);
  }
  if (has_audio) audio_device_close(&audio);
  renderer_destroy(renderer);
  window_close(&window);
  return status;
}
