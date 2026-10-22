#include "window.h"

#include "log.h"

#define CONTROLLER_DEAD_ZONE 0.15f

window_options_t window_default_options(const char* title) {
  window_options_t options = {title, 1280, 720, false, true, false};
  return options;
}

static void open_first_controller(window_t* window) {
  for (int i = 0; i < SDL_NumJoysticks(); i++) {
    if (!SDL_IsGameController(i)) continue;
    window->controller = SDL_GameControllerOpen(i);
    if (window->controller) {
      LOG_INFO("platform", "controller: %s",
               SDL_GameControllerName(window->controller));
      return;
    }
  }
}

static void read_sizes(window_t* window) {
  SDL_GetWindowSize(window->window, &window->width, &window->height);
  SDL_GL_GetDrawableSize(window->window, &window->drawable_width,
                         &window->drawable_height);
}

bool window_open(window_t* window, window_options_t options) {
  SDL_memset(window, 0, sizeof(*window));
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS) !=
      0) {
    LOG_ERROR("platform", "SDL_Init: %s", SDL_GetError());
    return false;
  }
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  Uint32 flags =
      SDL_WINDOW_OPENGL | SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_RESIZABLE;
  if (options.fullscreen) flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
  if (options.hidden) flags |= SDL_WINDOW_HIDDEN;
  window->window = SDL_CreateWindow(options.title, SDL_WINDOWPOS_CENTERED,
                                    SDL_WINDOWPOS_CENTERED, options.width,
                                    options.height, flags);
  if (!window->window) {
    LOG_ERROR("platform", "SDL_CreateWindow: %s", SDL_GetError());
    SDL_Quit();
    return false;
  }
  window->context = SDL_GL_CreateContext(window->window);
  if (!window->context) {
    LOG_ERROR("platform", "SDL_GL_CreateContext: %s", SDL_GetError());
    SDL_DestroyWindow(window->window);
    SDL_Quit();
    return false;
  }
  SDL_GL_MakeCurrent(window->window, window->context);
  if (SDL_GL_SetSwapInterval(options.vsync ? 1 : 0) != 0)
    LOG_WARN("platform", "vsync not available: %s", SDL_GetError());
  read_sizes(window);
  open_first_controller(window);
  LOG_INFO("platform", "window %dx%d, drawable %dx%d", window->width,
           window->height, window->drawable_width, window->drawable_height);
  return true;
}

void window_close(window_t* window) {
  if (window->controller) SDL_GameControllerClose(window->controller);
  if (window->context) SDL_GL_DeleteContext(window->context);
  if (window->window) SDL_DestroyWindow(window->window);
  SDL_memset(window, 0, sizeof(*window));
  SDL_Quit();
}

static void handle_key(window_events_t* events, SDL_Scancode key) {
  switch (key) {
    case SDL_SCANCODE_ESCAPE:
      events->quit = true;
      break;
    case SDL_SCANCODE_R:
      events->reset_vehicle = true;
      break;
    case SDL_SCANCODE_P:
      events->toggle_pause = true;
      break;
    case SDL_SCANCODE_C:
      events->toggle_camera = true;
      break;
    case SDL_SCANCODE_F1:
      events->toggle_debug = true;
      break;
    case SDL_SCANCODE_F11:
      events->toggle_fullscreen = true;
      break;
    case SDL_SCANCODE_M:
      events->toggle_map = true;
      break;
    case SDL_SCANCODE_T:
      events->toggle_weather = true;
      break;
    case SDL_SCANCODE_F:
      events->interact = true;
      break;
    case SDL_SCANCODE_BACKSPACE:
      events->back_to_menu = true;
      break;
    case SDL_SCANCODE_V:
      events->next_vehicle = true;
      break;
    case SDL_SCANCODE_N:
      events->toggle_mute = true;
      break;
    default:
      break;
  }
}

static void handle_button(window_events_t* events, Uint8 button) {
  switch (button) {
    case SDL_CONTROLLER_BUTTON_Y:
      events->reset_vehicle = true;
      break;
    case SDL_CONTROLLER_BUTTON_START:
      events->toggle_pause = true;
      break;
    case SDL_CONTROLLER_BUTTON_BACK:
      events->toggle_camera = true;
      break;
    default:
      break;
  }
}

window_events_t window_poll(window_t* window) {
  window_events_t events = {0};
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_QUIT:
        events.quit = true;
        break;
      case SDL_KEYDOWN:
        if (!event.key.repeat) handle_key(&events, event.key.keysym.scancode);
        break;
      case SDL_CONTROLLERBUTTONDOWN:
        handle_button(&events, event.cbutton.button);
        break;
      case SDL_CONTROLLERDEVICEADDED:
        if (!window->controller) open_first_controller(window);
        break;
      case SDL_CONTROLLERDEVICEREMOVED:
        if (window->controller) {
          SDL_GameControllerClose(window->controller);
          window->controller = NULL;
        }
        break;
      case SDL_WINDOWEVENT:
        if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED ||
            event.window.event == SDL_WINDOWEVENT_RESIZED) {
          read_sizes(window);
          events.resized = true;
        }
        break;
      default:
        break;
    }
  }
  return events;
}

menu_key_t window_poll_menu(window_t* window) {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_QUIT:
        return MENU_KEY_QUIT;
      case SDL_KEYDOWN:
        switch (event.key.keysym.scancode) {
          case SDL_SCANCODE_UP:
          case SDL_SCANCODE_W:
            return MENU_KEY_UP;
          case SDL_SCANCODE_DOWN:
          case SDL_SCANCODE_S:
            return MENU_KEY_DOWN;
          case SDL_SCANCODE_RETURN:
          case SDL_SCANCODE_KP_ENTER:
          case SDL_SCANCODE_SPACE:
            return MENU_KEY_SELECT;
          case SDL_SCANCODE_ESCAPE:
            return MENU_KEY_QUIT;
          default:
            break;
        }
        break;
      case SDL_CONTROLLERBUTTONDOWN:
        if (event.cbutton.button == SDL_CONTROLLER_BUTTON_DPAD_UP)
          return MENU_KEY_UP;
        if (event.cbutton.button == SDL_CONTROLLER_BUTTON_DPAD_DOWN)
          return MENU_KEY_DOWN;
        if (event.cbutton.button == SDL_CONTROLLER_BUTTON_A ||
            event.cbutton.button == SDL_CONTROLLER_BUTTON_START)
          return MENU_KEY_SELECT;
        break;
      case SDL_CONTROLLERDEVICEADDED:
        if (!window->controller) open_first_controller(window);
        break;
      case SDL_WINDOWEVENT:
        if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED ||
            event.window.event == SDL_WINDOWEVENT_RESIZED)
          read_sizes(window);
        break;
      default:
        break;
    }
  }
  return MENU_KEY_NONE;
}

static float axis(const SDL_GameController* controller,
                  SDL_GameControllerAxis which) {
  Sint16 raw =
      SDL_GameControllerGetAxis((SDL_GameController*)controller, which);
  return apply_dead_zone((float)raw / 32767.0f, CONTROLLER_DEAD_ZONE);
}

bool window_horn_held(const window_t* window) {
  const Uint8* keys = SDL_GetKeyboardState(NULL);
  if (keys[SDL_SCANCODE_H]) return true;
  return window->controller &&
         SDL_GameControllerGetButton((SDL_GameController*)window->controller,
                                     SDL_CONTROLLER_BUTTON_B);
}

driving_input_t window_driving_input(const window_t* window) {
  driving_input_t keyboard =
      driving_input_from_keys(SDL_GetKeyboardState(NULL));
  if (!window->controller) return keyboard;
  driving_input_t pad = {0};
  pad.steering = axis(window->controller, SDL_CONTROLLER_AXIS_LEFTX);
  pad.throttle = axis(window->controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
  pad.brake = axis(window->controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
  pad.handbrake =
      SDL_GameControllerGetButton((SDL_GameController*)window->controller,
                                  SDL_CONTROLLER_BUTTON_A) != 0;
  return driving_input_merge(keyboard, pad);
}

void window_swap(window_t* window) { SDL_GL_SwapWindow(window->window); }

void window_toggle_fullscreen(window_t* window) {
  Uint32 flags = SDL_GetWindowFlags(window->window);
  bool fullscreen = (flags & SDL_WINDOW_FULLSCREEN_DESKTOP) != 0;
  SDL_SetWindowFullscreen(window->window,
                          fullscreen ? 0 : SDL_WINDOW_FULLSCREEN_DESKTOP);
  read_sizes(window);
}

double platform_seconds(void) {
  return (double)SDL_GetPerformanceCounter() /
         (double)SDL_GetPerformanceFrequency();
}
