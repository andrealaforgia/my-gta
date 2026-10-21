/* Drives the car on the test track for a few seconds through a hidden
 * window, renders one frame, saves it as a BMP and checks it is not
 * blank. Needs a GPU, so it is not part of `make test`. */
#include <SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driving_stage.h"
#include "log.h"
#include "renderer.h"
#include "window.h"

static int save_bmp(const char* path, int width, int height, uint8_t* rgba) {
  SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(0, width, height, 32,
                                                        SDL_PIXELFORMAT_RGBA32);
  if (!surface) return 0;
  for (int y = 0; y < height; y++)
    memcpy((uint8_t*)surface->pixels + y * surface->pitch,
           rgba + (height - 1 - y) * width * 4, (size_t)width * 4);
  int ok = SDL_SaveBMP(surface, path) == 0;
  SDL_FreeSurface(surface);
  return ok;
}

int main(int argc, char* argv[]) {
  const char* path = argc > 1 ? argv[1] : "snapshot.bmp";
  float seconds = argc > 2 ? (float)atof(argv[2]) : 4.0f;
  float steering = argc > 3 ? (float)atof(argv[3]) : 0.0f;
  const char* city = argc > 4 && argv[4][0] != '-' ? argv[4] : NULL;
  /* Optional overhead view: height in metres above the car, 0 for the
   * chase camera. */
  float overhead = argc > 5 ? (float)atof(argv[5]) : 0.0f;
  window_options_t options = window_default_options("snapshot");
  options.hidden = true;
  options.vsync = false;
  options.width = 960;
  options.height = 540;
  window_t window;
  if (!window_open(&window, options)) return EXIT_FAILURE;
  renderer_t* renderer =
      renderer_create(window.drawable_width, window.drawable_height);
  if (!renderer) return EXIT_FAILURE;
  stage_options_t stage_options = {
      city ? SCENE_CITY : SCENE_TEST_TRACK,
      city,
      NULL,
      getenv("SNAPSHOT_NO_DEBUG") == NULL,
      true,
      getenv("SNAPSHOT_HOUR") ? (float)atof(getenv("SNAPSHOT_HOUR")) : 10.0f,
      0.0f,
      getenv("SNAPSHOT_TRAFFIC") ? atoi(getenv("SNAPSHOT_TRAFFIC")) : 0,
      getenv("SNAPSHOT_PEOPLE") ? atoi(getenv("SNAPSHOT_PEOPLE")) : 0,
      getenv("SNAPSHOT_RAIN") ? (float)atof(getenv("SNAPSHOT_RAIN")) : 0.0f};
  driving_stage_t stage;
  if (!driving_stage_init(&stage, &window, renderer, NULL, stage_options))
    return EXIT_FAILURE;

  /* SNAPSHOT_VEHICLE=n swaps to the nth vehicle in the fleet, as V does. */
  for (int v = getenv("SNAPSHOT_VEHICLE") ? atoi(getenv("SNAPSHOT_VEHICLE"))
                                          : 0;
       v > 0; v--)
    driving_stage_next_vehicle(&stage);

  const char* centre = getenv("SNAPSHOT_CENTRE");
  if (centre) {
    float cx = 0, cz = 0, yaw_degrees = 0;
    sscanf(centre, "%f,%f,%f", &cx, &cz, &yaw_degrees);
    /* Drop the car onto whatever ground is there, hills included. */
    ray_t down = {vec3(cx, 300.0f, cz), vec3(0, -1, 0)};
    ray_cast_result_t hit;
    float y = 1.0f;
    if (physics_ray_cast(stage.physics, down, 600.0f, COLLISION_LAYER_GROUND,
                         &hit))
      y = hit.hit.point.y + 1.0f;
    vehicle_set_spawn(&stage.vehicle, vec3(cx, y, cz),
                      quat_from_yaw(yaw_degrees * (float)M_PI / 180.0f));
    vehicle_reset(&stage.vehicle, stage.physics);
  }
  driving_input_t input = {1.0f, 0.0f, steering, false};
  int steps = (int)(seconds * 120);
  for (int i = 0; i < steps; i++) {
    driving_stage_step(&stage, input, 1.0f / 120.0f);
    if (i % 2 == 0) {
      const rigid_body_t* body =
          physics_body(stage.physics, stage.vehicle.body);
      chase_camera_update(&stage.camera,
                          transform_at(body->position, body->rotation),
                          body->velocity, stage.physics, 1.0f / 60.0f);
    }
  }
  stage.fps = 60;
  stage.view_mode = getenv("SNAPSHOT_VIEW") ? atoi(getenv("SNAPSHOT_VIEW")) : 0;
  stage.map_open = getenv("SNAPSHOT_MAP") != NULL;
  if (overhead > 0) {
    const rigid_body_t* body = physics_body(stage.physics, stage.vehicle.body);
    /* A hair south of straight down keeps north at the top of the frame. */
    stage.camera.position = vec3_add(body->position, vec3(0, overhead, 0.01f));
    stage.camera.look_target = body->position;
    stage.scene.environment.fog_start = 5000.0f; /* a map, not a view */
    stage.scene.environment.fog_end = 6000.0f;
    stage.scene.environment.shadow_radius = 0;
  }
  if (getenv("SNAPSHOT_LOOKUP")) {
    stage.camera.look_target = vec3_add(
        stage.camera.position, vec3(20, atof(getenv("SNAPSHOT_LOOKUP")), -60));
  }
  driving_stage_render(&stage);
  /* Time repeated frames for a first performance figure. */
  const int timed_frames = 60;
  double started = platform_seconds();
  for (int i = 0; i < timed_frames; i++) {
    driving_stage_render(&stage);
    window_swap(&window);
  }
  double frame_ms = (platform_seconds() - started) * 1000.0 / timed_frames;
  driving_stage_render(&stage);

  int width = window.drawable_width, height = window.drawable_height;
  uint8_t* pixels = malloc((size_t)width * height * 4);
  renderer_read_pixels(renderer, pixels, (size_t)width * height * 4);
  long distinct = 0;
  for (int i = 1; i < width * height; i++)
    if (memcmp(pixels + i * 4, pixels + (i - 1) * 4, 3) != 0) distinct++;
  int status = distinct > width * height / 20 ? EXIT_SUCCESS : EXIT_FAILURE;
  if (status != EXIT_SUCCESS) LOG_ERROR("snapshot", "the frame looks blank");
  if (!save_bmp(path, width, height, pixels)) status = EXIT_FAILURE;
  render_stats_t stats = renderer_stats(renderer);
  LOG_INFO("snapshot", "shadow map coverage %.3f",
           renderer_shadow_coverage(renderer));
  if (getenv("SNAPSHOT_SHADOW_MAP")) {
    size_t count = 2048u * 2048u;
    float* depths = malloc(count * sizeof(float));
    int size = renderer_read_shadow_map(renderer, depths, count);
    if (size > 0) {
      uint8_t* grey = malloc(count * 4);
      for (size_t i = 0; i < count; i++) {
        uint8_t v = (uint8_t)(255.0f * (1.0f - depths[i]) * 4.0f > 255
                                  ? 255
                                  : 255.0f * (1.0f - depths[i]) * 4.0f);
        grey[i * 4] = grey[i * 4 + 1] = grey[i * 4 + 2] = v;
        grey[i * 4 + 3] = 255;
      }
      save_bmp(getenv("SNAPSHOT_SHADOW_MAP"), size, size, grey);
      free(grey);
    }
    free(depths);
  }
  LOG_INFO("snapshot",
           "%s: %dx%d, %d draws, %d triangles, %.2f ms per frame, car at %.1f "
           "km/h after %.1f s",
           path, width, height, stats.draw_calls, stats.triangles, frame_ms,
           vehicle_speed_kmh(&stage.vehicle), seconds);
  free(pixels);
  driving_stage_destroy(&stage);
  renderer_destroy(renderer);
  window_close(&window);
  return status;
}
