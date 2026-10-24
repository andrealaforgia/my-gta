#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "audio_device.h"
#include "benchmark.h"
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
  bool scene_chosen; /* on the command line, so no menu */
  bool benchmark;
  const char* benchmark_baseline;
  const char* benchmark_output;
  char city_path[256]; /* the package for --city=NAME */
} options_t;

#define CITIES_DIRECTORY "assets_cooked/cities"

static bool valid_benchmark_numbers(int argc, char* argv[]) {
  const char* integers[] = {
      "--width=", "--height=", "--traffic=", "--pedestrians="};
  const char* decimals[] = {"--time=", "--hour-seconds="};
  for (int i = 1; i < argc; i++) {
    for (size_t k = 0; k < sizeof(integers) / sizeof(integers[0]); k++) {
      size_t length = strlen(integers[k]);
      if (strncmp(argv[i], integers[k], length) != 0) continue;
      const char* value = argv[i] + length;
      char* end;
      errno = 0;
      long parsed = strtol(value, &end, 10);
      if (end == value || *end || errno || parsed < INT_MIN || parsed > INT_MAX)
        return false;
    }
    for (size_t k = 0; k < sizeof(decimals) / sizeof(decimals[0]); k++) {
      size_t length = strlen(decimals[k]);
      if (strncmp(argv[i], decimals[k], length) != 0) continue;
      const char* value = argv[i] + length;
      char* end;
      errno = 0;
      double parsed = strtod(value, &end);
      if (end == value || *end || errno || !isfinite(parsed)) return false;
    }
  }
  return true;
}

static void print_help(void) {
  printf(
      "gta-like: drive a car on a test track or in a generated city.\n\n"
      "  --scene=track            the test track (default)\n"
      "  --scene=city:<package>   a cooked city package directory\n"
      "  --city=<name>            a cooked city by name, such as pordenone, "
      "barletta or pessano_con_bornago\n"
      "                           (with no scene given, a menu offers them)\n"
      "  --vehicle=<file>         vehicle parameters "
      "(assets/vehicles/hatchback.cfg)\n"
      "  --width=N --height=N     window size (1280x720)\n"
      "  --fullscreen             borderless fullscreen\n"
      "  --no-vsync               do not wait for the display\n"
      "  --no-audio               do not open an audio device\n"
      "  --bench                  30 simulated seconds of city flyover, no "
      "audio or "
      "vsync\n"
      "  --bench-baseline=<file>   fail above a 25%% performance regression\n"
      "  --bench-output=<file>     save the report as a reusable baseline\n"
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
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--bench") == 0 &&
        !valid_benchmark_numbers(argc, argv)) {
      fprintf(stderr, "--bench requires a city and valid workload options\n");
      return false;
    }
  }
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
    } else if (strcmp(argument, "--bench") == 0) {
      options->benchmark = true;
    } else if (strncmp(argument, "--bench-baseline=", 17) == 0) {
      options->benchmark_baseline = argument + 17;
    } else if (strncmp(argument, "--bench-output=", 15) == 0) {
      options->benchmark_output = argument + 15;
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
  if ((options->benchmark_baseline || options->benchmark_output) &&
      !options->benchmark) {
    fprintf(stderr, "benchmark file options require --bench\n");
    return false;
  }
  if (options->benchmark) {
    struct stat baseline_file, output_file;
    if (options->benchmark_baseline && options->benchmark_output &&
        (strcmp(options->benchmark_baseline, options->benchmark_output) == 0 ||
         (stat(options->benchmark_baseline, &baseline_file) == 0 &&
          stat(options->benchmark_output, &output_file) == 0 &&
          baseline_file.st_dev == output_file.st_dev &&
          baseline_file.st_ino == output_file.st_ino))) {
      fprintf(stderr, "benchmark output must not overwrite its baseline\n");
      return false;
    }
    if (!options->scene_chosen || options->stage.scene != SCENE_CITY ||
        !options->stage.city_package[0] ||
        (options->benchmark_baseline && !options->benchmark_baseline[0]) ||
        (options->benchmark_output && !options->benchmark_output[0]) ||
        !isfinite(options->stage.start_hour) || options->stage.start_hour < 0 ||
        options->stage.start_hour >= 24 ||
        !isfinite(options->stage.hour_seconds) ||
        options->stage.hour_seconds < 0 || options->stage.traffic_cars < 0 ||
        options->stage.pedestrians < 0) {
      fprintf(stderr, "--bench requires a city and valid workload options\n");
      return false;
    }
    options->window.vsync = false;
    options->window.hidden = true;
    options->no_audio = true;
  }
  return true;
}

static int run_benchmark(const options_t* options, window_t* window,
                         renderer_t* renderer) {
  FILE* output = options->benchmark_output
                     ? fopen(options->benchmark_output, "w")
                     : stdout;
  if (!output) {
    fprintf(stderr, "cannot write benchmark output %s\n",
            options->benchmark_output);
    return EXIT_FAILURE;
  }
  driving_stage_t stage;
  bool success =
      driving_stage_init(&stage, window, renderer, NULL, options->stage);
  benchmark_report_t report = {0};
  if (success) {
    success = benchmark_run(&stage, BENCHMARK_FRAMES, BENCHMARK_WARMUP_FRAMES,
                            options->benchmark_baseline, output, &report);
    if (output != stdout && report.frame_average_ms > 0)
      success = benchmark_write(stdout, &report) && success;
  }
  if (output != stdout && fclose(output) != 0) success = false;
  driving_stage_destroy(&stage);
  return success ? EXIT_SUCCESS : EXIT_FAILURE;
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
  if (options.benchmark) {
    int status = run_benchmark(&options, &window, renderer);
    renderer_destroy(renderer);
    window_close(&window);
    return status;
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
