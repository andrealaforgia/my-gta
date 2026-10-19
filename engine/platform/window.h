/**
 * @file window.h
 * @brief The SDL window with its OpenGL context, event polling, keyboard
 * and controller state, and the high-resolution clock. The only engine
 * module above the renderer that talks to SDL's video subsystem.
 */
#ifndef ENGINE_PLATFORM_WINDOW_H_
#define ENGINE_PLATFORM_WINDOW_H_

#include <SDL.h>
#include <stdbool.h>

#include "input.h"

typedef struct {
  const char* title;
  int width;
  int height;
  bool fullscreen;
  bool vsync;
  bool hidden; /* for tests and snapshots */
} window_options_t;

typedef struct {
  bool quit;
  bool resized;
  bool reset_vehicle;
  bool toggle_pause;
  bool toggle_camera;
  bool toggle_debug;
  bool toggle_fullscreen;
  bool toggle_mute;
} window_events_t;

typedef struct {
  SDL_Window* window;
  SDL_GLContext context;
  SDL_GameController* controller;
  int width;
  int height;
  int drawable_width;
  int drawable_height;
} window_t;

window_options_t window_default_options(const char* title);
bool window_open(window_t* window, window_options_t options);
void window_close(window_t* window);
window_events_t window_poll(window_t* window);
driving_input_t window_driving_input(const window_t* window);
void window_swap(window_t* window);
void window_toggle_fullscreen(window_t* window);
double platform_seconds(void);

#endif  // ENGINE_PLATFORM_WINDOW_H_
