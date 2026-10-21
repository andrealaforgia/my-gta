#include "city_menu.h"

#include <stdio.h>

const char* city_menu_row_title(const city_entry_t* cities, size_t count,
                                size_t row) {
  if (row == 0) return "Test track";
  return row <= count ? cities[row - 1].title : "";
}

size_t city_menu_move(size_t row, size_t rows, menu_key_t key) {
  if (rows == 0) return 0;
  if (key == MENU_KEY_UP) return (row + rows - 1) % rows;
  if (key == MENU_KEY_DOWN) return (row + 1) % rows;
  return row;
}

static void draw_menu(renderer_t* renderer, window_t* window,
                      const city_entry_t* cities, size_t count, size_t row) {
  render_camera_t camera = {vec3(0, 2, 0), vec3(0, 2, -10), vec3(0, 1, 0), 1.0f,
                            0.1f,          100.0f};
  render_environment_t environment = {0};
  environment.direction = vec3(0.3f, 0.8f, 0.4f);
  environment.color = vec3(1, 1, 1);
  environment.ambient = 0.6f;
  environment.sky_color = vec3(0.62f, 0.74f, 0.88f);
  environment.zenith_color = vec3(0.25f, 0.45f, 0.75f);
  environment.fog_start = 50.0f;
  environment.fog_end = 100.0f;
  renderer_begin_frame(renderer, camera, environment);
  float scale = (float)window->drawable_height / 180.0f;
  if (scale < 2.0f) scale = 2.0f;
  float x = (float)window->drawable_width * 0.5f - 14.0f * 6.0f * scale * 0.5f;
  float y = (float)window->drawable_height * 0.28f;
  renderer_draw_text(renderer, x, y, scale * 1.4f, vec3(1, 1, 1),
                     "CHOOSE WHERE TO DRIVE");
  y += scale * 16.0f;
  for (size_t i = 0; i <= count; i++) {
    char line[80];
    snprintf(line, sizeof(line), "%s %s", i == row ? ">" : " ",
             city_menu_row_title(cities, count, i));
    for (char* c = line; *c; c++)
      if (*c >= 'a' && *c <= 'z') *c = (char)(*c - 'a' + 'A');
    renderer_draw_text(
        renderer, x, y, scale,
        i == row ? vec3(1.0f, 0.85f, 0.3f) : vec3(0.9f, 0.9f, 0.9f), line);
    y += scale * 10.0f;
  }
  renderer_draw_text(renderer, x, y + scale * 8.0f, scale * 0.8f,
                     vec3(0.8f, 0.8f, 0.8f),
                     "UP DOWN CHOOSE  ENTER DRIVE  ESC QUIT");
  renderer_end_frame(renderer);
}

int city_menu_run(window_t* window, renderer_t* renderer,
                  const city_entry_t* cities, size_t count, int selected) {
  size_t rows = count + 1;
  size_t row = selected >= 0 && (size_t)selected < rows ? (size_t)selected : 0;
  for (;;) {
    menu_key_t key;
    while ((key = window_poll_menu(window)) != MENU_KEY_NONE) {
      if (key == MENU_KEY_QUIT) return -2;
      if (key == MENU_KEY_SELECT) return (int)row - 1;
      row = city_menu_move(row, rows, key);
    }
    renderer_resize(renderer, window->drawable_width, window->drawable_height);
    draw_menu(renderer, window, cities, count, row);
    window_swap(window);
  }
}
