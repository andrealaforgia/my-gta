/* Renders one frame of a cube on a grid through a hidden window and saves
 * it as a BMP, then checks the frame is not blank. Needs a GPU. */
#include <SDL.h>
#include <stdio.h>
#include <stdlib.h>

#include "log.h"
#include "mesh_data.h"
#include "renderer.h"
#include "window.h"

static int save_bmp(const char* path, int width, int height, uint8_t* rgba) {
  /* GL rows are bottom-up; flip into the surface. */
  SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(0, width, height, 32, SDL_PIXELFORMAT_RGBA32);
  if (!surface) return 0;
  for (int y = 0; y < height; y++)
    memcpy((uint8_t*)surface->pixels + y * surface->pitch, rgba + (height - 1 - y) * width * 4,
           (size_t)width * 4);
  int ok = SDL_SaveBMP(surface, path) == 0;
  SDL_FreeSurface(surface);
  return ok;
}

int main(int argc, char* argv[]) {
  const char* path = argc > 1 ? argv[1] : "snapshot.bmp";
  window_options_t options = window_default_options("snapshot");
  options.hidden = true;
  options.vsync = false;
  options.width = 640;
  options.height = 360;
  window_t window;
  if (!window_open(&window, options)) return EXIT_FAILURE;
  renderer_t* renderer = renderer_create(window.drawable_width, window.drawable_height);
  if (!renderer) return EXIT_FAILURE;

  mesh_data_t cube = mesh_data_create();
  mesh_data_add_box(&cube, vec3(0, 1, 0), vec3(1, 1, 1));
  mesh_handle_t cube_mesh = renderer_upload_mesh(renderer, &cube);
  mesh_data_t grid = mesh_data_create();
  mesh_data_add_grid(&grid, 20, 2);
  mesh_handle_t grid_mesh = renderer_upload_mesh(renderer, &grid);

  render_camera_t camera = {vec3(6, 4, 8), vec3(0, 1, 0), vec3(0, 1, 0), 1.0f, 0.1f, 200.0f};
  render_environment_t environment = {vec3(0.4f, 1.0f, 0.3f), vec3(1, 0.95f, 0.85f), 0.35f,
                                      vec3(0.55f, 0.7f, 0.9f), 60.0f, 180.0f};
  renderer_begin_frame(renderer, camera, environment);
  renderer_draw_mesh(renderer, grid_mesh, material_plain(vec3(0.35f, 0.4f, 0.35f)), mat4_identity());
  renderer_draw_mesh(renderer, cube_mesh, material_plain(vec3(0.8f, 0.2f, 0.15f)), mat4_identity());
  renderer_draw_line(renderer, vec3(0, 0, 0), vec3(3, 0, 0), vec3(1, 0, 0));
  renderer_draw_line(renderer, vec3(0, 0, 0), vec3(0, 3, 0), vec3(0, 1, 0));
  renderer_draw_line(renderer, vec3(0, 0, 0), vec3(0, 0, 3), vec3(0, 0, 1));
  renderer_draw_text(renderer, 10, 10, 3, vec3(1, 1, 1), "SNAPSHOT 123 KM/H");
  renderer_end_frame(renderer);

  int width = window.drawable_width, height = window.drawable_height;
  uint8_t* pixels = malloc((size_t)width * height * 4);
  renderer_read_pixels(renderer, pixels, (size_t)width * height * 4);
  int status = EXIT_SUCCESS;
  uint8_t* centre = pixels + ((height / 2) * width + width / 2) * 4;
  if (!(centre[0] > centre[1] && centre[0] > centre[2])) {
    LOG_ERROR("snapshot", "centre pixel is not the red cube: %d %d %d", centre[0], centre[1], centre[2]);
    status = EXIT_FAILURE;
  }
  if (!save_bmp(path, width, height, pixels)) status = EXIT_FAILURE;
  render_stats_t stats = renderer_stats(renderer);
  LOG_INFO("snapshot", "%s: %dx%d, %d draw calls, %d triangles", path, width, height,
           stats.draw_calls, stats.triangles);
  free(pixels);
  mesh_data_free(&cube);
  mesh_data_free(&grid);
  renderer_destroy(renderer);
  window_close(&window);
  return status;
}
