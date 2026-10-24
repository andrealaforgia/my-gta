/* Drives the car on the test track for a few seconds through a hidden
 * window, renders one frame, saves it as a BMP and checks it is not
 * blank. Needs a GPU, so it is not part of `make test`. */
#include <SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "air_stage.h"
#include "city_menu.h"
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
  if (getenv("SNAPSHOT_MENU")) {
    /* The start menu, with the second row chosen. */
    city_entry_t cities[CITY_LIST_MAX];
    size_t count = city_list("assets_cooked/cities", cities, CITY_LIST_MAX);
    city_menu_draw(renderer, &window, cities, count, count > 1 ? 2 : 1);
    int w = window.drawable_width, h = window.drawable_height;
    uint8_t* menu_pixels = malloc((size_t)w * h * 4);
    renderer_read_pixels(renderer, menu_pixels, (size_t)w * h * 4);
    int saved = save_bmp(path, w, h, menu_pixels);
    free(menu_pixels);
    return saved ? EXIT_SUCCESS : EXIT_FAILURE;
  }
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
      getenv("SNAPSHOT_RAIN") ? (float)atof(getenv("SNAPSHOT_RAIN")) : 0.0f,
      false,
      getenv("SNAPSHOT_AUTOPLAY") != NULL};
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
  driving_input_t input = {1.0f, 0.0f, steering, false, false};
  /* SNAPSHOT_FOOT gets out of the car after a short drive and walks. */
  /* SNAPSHOT_FLY=n then steps into the nth aircraft at the airport and takes
   * off with a fixed script: full power, the nose up after nine seconds. */
  int fly = getenv("SNAPSHOT_FLY") ? atoi(getenv("SNAPSHOT_FLY")) : -1;
  bool on_foot = getenv("SNAPSHOT_FOOT") != NULL || fly >= 0;
  if (on_foot) {
    driving_input_t brake = {0.0f, 1.0f, 0.0f, true, false};
    for (int i = 0; i < 120 * 3; i++)
      driving_stage_step(&stage, brake, 1.0f / 120.0f);
    driving_stage_interact(&stage);
    if (fly >= 0 && (size_t)fly < stage.aircraft_count) {
      aircraft_t* craft = &stage.aircraft[fly];
      const city_airport_t* airport = &stage.scene.package->airport;
      if (craft->kind == AIRCRAFT_PLANE && airport->present) {
        /* A plane starts at the end of the runway, nose along it. */
        vec3_t axis = vec3(-sinf(airport->yaw), 0, -cosf(airport->yaw));
        float x = airport->x - axis.x * 420.0f,
              z = airport->z - axis.z * 420.0f;
        ray_t down = {vec3(x, 400.0f, z), vec3(0, -1, 0)};
        ray_cast_result_t hit;
        if (physics_ray_cast(stage.physics, down, 800.0f,
                             COLLISION_LAYER_GROUND, &hit))
          craft->position = vec3(x, hit.hit.point.y, z);
        craft->yaw = airport->yaw;
      }
      vec3_t at = craft->position;
      stage.walker.position = vec3(at.x + 3.0f, at.y, at.z);
      driving_stage_interact(&stage);
    }
    /* SNAPSHOT_SHOOT fires the pistol, so the shot shows the tracer. */
    if (getenv("SNAPSHOT_SHOOT")) {
      driving_input_t stand = {0.0f, 0.0f, 0.0f, false, false};
      for (int i = 0; i < 60; i++)
        driving_stage_step(&stage, stand, 1.0f / 120.0f);
      driving_stage_fire(&stage);
    }
    /* SNAPSHOT_ENTER then gets back in, so the shot is mid-climb. */
    if (getenv("SNAPSHOT_ENTER")) {
      driving_input_t stand = {0.0f, 0.0f, 0.0f, false, false};
      for (int i = 0; i < 120; i++)
        driving_stage_step(&stage, stand, 1.0f / 120.0f);
      driving_stage_interact(&stage);
    }
  }
  /* SNAPSHOT_FIRE=1 wrecks the car: flames after 0 s, a blast after 7 s. */
  if (getenv("SNAPSHOT_FIRE")) stage.vehicle.damage = 1.0f;
  int steps = (int)(seconds * 120);
  /* SNAPSHOT_STUCK reports the cars that stood still for long. */
  bool watch_stuck = getenv("SNAPSHOT_STUCK") != NULL && stage.traffic;
  size_t watched = watch_stuck ? traffic_car_count(stage.traffic) : 0;
  float* stood = watched ? calloc(watched, sizeof(float)) : NULL;
  float* longest = watched ? calloc(watched, sizeof(float)) : NULL;
  /* SNAPSHOT_CRIME starts a police chase and reports it every five seconds. */
  bool crime = getenv("SNAPSHOT_CRIME") != NULL && stage.traffic;
  if (crime) wanted_crime(&stage.wanted);
  /* SNAPSHOT_SOAK drives about at random, getting out and in, stealing cars
   * and breaking in changes of vehicle, and checks that nothing goes wrong. */
  bool soak = getenv("SNAPSHOT_SOAK") != NULL;
  /* SNAPSHOT_JERK reports traffic cars that jump or spin between steps. */
  bool jerk = getenv("SNAPSHOT_JERK") != NULL && stage.traffic;
  size_t jerk_count = jerk ? traffic_car_count(stage.traffic) : 0;
  traffic_car_t* jerk_before =
      jerk_count ? calloc(jerk_count, sizeof(traffic_car_t)) : NULL;
  int jerk_events = 0, overlaps = 0;
  /* SNAPSHOT_TRACE=file writes where every traffic car is, twenty times a
   * second, for plotting. */
  FILE* trace = getenv("SNAPSHOT_TRACE") && stage.traffic
                    ? fopen(getenv("SNAPSHOT_TRACE"), "w")
                    : NULL;
  unsigned soak_random = 12345;
  /* SNAPSHOT_AUTOPLAY=n starts the script at act n (0 is the beginning). */
  if (stage.autoplay.enabled && atoi(getenv("SNAPSHOT_AUTOPLAY")) > 0)
    stage.autoplay.act = (autoplay_act_t)atoi(getenv("SNAPSHOT_AUTOPLAY"));
  if (stage.autoplay.enabled && stage.autoplay.act >= ACT_BOARD_PLANE) {
    /* From the aeroplane on, begin on foot at the airport's gate. */
    const city_airport_t* gate = &stage.scene.package->airport;
    walker_init(&stage.walker, vec3(gate->gate_x, gate->gate_y, gate->gate_z),
                0.0f);
    stage.on_foot = true;
  }
  autoplay_act_t last_act = (autoplay_act_t)-1;
  for (int i = 0; i < steps; i++) {
    if (stage.autoplay.enabled && i % 2 == 0) {
      /* Once a frame, as in the game, the input held for the steps between. */
      input = autoplay_step(&stage.autoplay, &stage, 1.0f / 60.0f);
      vec3_t at =
          stage.piloting > 0
              ? stage.aircraft[stage.piloting - 1].position
              : (stage.on_foot ? stage.walker.position
                               : physics_body(stage.physics, stage.vehicle.body)
                                     ->position);
      if (stage.autoplay.act != last_act || i % 1200 == 0) {
        printf(
            "t=%.0f s act %s at %.0f %.0f %.0f foot %d piloting %d vehicle %d "
            "stars %d\n",
            (float)i / 120.0f, autoplay_label(&stage.autoplay), at.x, at.y,
            at.z, stage.on_foot, stage.piloting, stage.vehicle_index,
            stage.wanted.stars);
        last_act = stage.autoplay.act;
      }
    }
    if (soak && i % 60 == 0) {
      soak_random = soak_random * 1664525u + 1013904223u;
      unsigned r = soak_random >> 8;
      input.throttle = (r % 10) < 7 ? 1.0f : 0.0f;
      input.brake = (r % 10) == 9 ? 1.0f : 0.0f;
      input.steering = ((float)((r >> 4) % 200) - 100.0f) / 100.0f;
      input.handbrake = (r >> 12) % 17 == 0;
      if ((r >> 3) % 40 == 0) driving_stage_interact(&stage);
      if ((r >> 3) % 97 == 1) driving_stage_next_vehicle(&stage);
      if ((r >> 5) % 3 == 0) driving_stage_fire(&stage);
      if ((r >> 6) % 90 == 0) stage.vehicle.damage = 1.0f;
    }
    if (fly >= 0 && stage.piloting > 0) {
      float t = (float)i / 120.0f;
      bool plane = stage.aircraft[stage.piloting - 1].kind == AIRCRAFT_PLANE;
      input.throttle = 0, input.brake = 0, input.handbrake = true;
      input.steering = steering;
      if (plane && t > 9.0f) input.brake = t < 16.0f ? 1.0f : 0.0f;
      if (!plane && t > 8.0f) input.throttle = 1.0f;
    }
    driving_stage_step(&stage, input, 1.0f / 120.0f);
    if (trace && i % 6 == 0) {
      for (size_t k = 0; k < traffic_car_count(stage.traffic); k++) {
        traffic_car_t now = traffic_car(stage.traffic, k);
        fprintf(trace, "%zu %.2f %.2f %.3f %.2f\n", k, now.position.x,
                now.position.z, now.yaw, now.speed);
      }
    }
    if (jerk && i % 12 == 0) {
      /* Cars on top of one another (centres under 1.8 m apart). */
      for (size_t k = 0; k < jerk_count; k++)
        for (size_t m = k + 1; m < jerk_count; m++) {
          traffic_car_t a = traffic_car(stage.traffic, k),
                        b = traffic_car(stage.traffic, m);
          if (hypotf(a.position.x - b.position.x, a.position.z - b.position.z) <
              1.8f) {
            if (overlaps < 40)
              printf(
                  "overlap at %.1f s: cars %zu (route %d speed %.1f variant "
                  "%d) "
                  "and %zu (route %d speed %.1f variant %d) at %.1f %.1f / "
                  "%.1f "
                  "%.1f yaws %.2f %.2f\n",
                  (float)i / 120.0f, k, a.route, a.speed, a.variant, m, b.route,
                  b.speed, b.variant, a.position.x, a.position.z, b.position.x,
                  b.position.z, a.yaw, b.yaw);
            overlaps++;
          }
        }
    }
    if (jerk) {
      for (size_t k = 0; k < jerk_count; k++) {
        traffic_car_t now = traffic_car(stage.traffic, k);
        if (i > 0) {
          float moved = hypotf(now.position.x - jerk_before[k].position.x,
                               now.position.z - jerk_before[k].position.z);
          float turned = now.yaw - jerk_before[k].yaw;
          turned -= 6.2831853f * floorf(turned / 6.2831853f + 0.5f);
          if ((moved > 1.0f || fabsf(turned) > 0.06f) && jerk_events < 25) {
            printf(
                "jerk at %.1f s: car %zu variant %d route %d moved %.2f m "
                "turned %.3f rad speed %.1f\n",
                (float)i / 120.0f, k, now.variant, now.route, moved, turned,
                now.speed);
          }
          if (moved > 1.0f || fabsf(turned) > 0.06f) jerk_events++;
        }
        jerk_before[k] = now;
      }
    }
    if (soak) {
      vec3_t at =
          stage.on_foot
              ? stage.walker.position
              : physics_body(stage.physics, stage.vehicle.body)->position;
      if (!isfinite(at.x) || !isfinite(at.y) || !isfinite(at.z) ||
          at.y < -50.0f || at.y > 500.0f) {
        printf("SOAK FAILED at %d s: position %.1f %.1f %.1f\n", i / 120, at.x,
               at.y, at.z);
        return EXIT_FAILURE;
      }
      for (size_t k = 0; stage.traffic && k < traffic_car_count(stage.traffic);
           k++) {
        traffic_car_t car = traffic_car(stage.traffic, k);
        if (!isfinite(car.position.x) || !isfinite(car.position.z) ||
            car.speed > 60.0f) {
          printf("SOAK FAILED at %d s: traffic car %zu\n", i / 120, k);
          return EXIT_FAILURE;
        }
      }
    }
    if (crime && i % 240 == 0) {
      const rigid_body_t* me = physics_body(stage.physics, stage.vehicle.body);
      float chaser = 0;
      for (size_t k = 0; k < traffic_car_count(stage.traffic); k++)
        if (traffic_car(stage.traffic, k).pursuing)
          chaser = traffic_car(stage.traffic, k).speed;
      printf(
          "t=%d stars %d pursuers %d nearest %.0f m at %.0f m/s, player %.0f "
          "km/h '%s'\n",
          i / 120, stage.wanted.stars, traffic_pursuer_count(stage.traffic),
          traffic_nearest_pursuer(stage.traffic, me->position), chaser,
          vehicle_speed_kmh(&stage.vehicle), stage.notice);
    }
    if (watched && i % 120 == 0) {
      for (size_t k = 0; k < watched; k++) {
        traffic_car_t car = traffic_car(stage.traffic, k);
        stood[k] = car.speed < 0.1f ? stood[k] + 1.0f : 0.0f;
        if (stood[k] > longest[k]) longest[k] = stood[k];
      }
    }
    if (i % 2 == 0 && stage.piloting > 0) {
      air_stage_follow(&stage, 1.0f / 60.0f);
    } else if (i % 2 == 0) {
      const rigid_body_t* body =
          physics_body(stage.physics, stage.vehicle.body);
      chase_camera_update(&stage.camera,
                          transform_at(body->position, body->rotation),
                          body->velocity, stage.physics, 1.0f / 60.0f);
    }
  }
  for (size_t k = 0; k < watched; k++) {
    if (stood[k] < 40.0f) continue;
    traffic_car_t car = traffic_car(stage.traffic, k);
    printf("stuck %zu variant %d route %d at %.1f %.1f for %.0f s\n", k,
           car.variant, car.route, car.position.x, car.position.z, stood[k]);
  }
  free(stood);
  free(longest);
  if (jerk) printf("overlap samples: %d\n", overlaps);
  free(jerk_before);
  if (soak) printf("soak done: stars now %d\n", stage.wanted.stars);
  if (fly >= 0) {
    size_t loaded = 0;
    for (size_t k = 0; k < stage.scene.package->chunk_count; k++)
      loaded += stage.scene.chunk_states[k].loaded;
    printf("chunks loaded %zu of %zu, camera at %.0f %.0f %.0f, drawn %d\n",
           loaded, stage.scene.package->chunk_count, stage.camera.position.x,
           stage.camera.position.y, stage.camera.position.z, stage.last_drawn);
  }
  if (fly >= 0)
    printf(
        "flight: piloting %d, %.0f m up, %.0f km/h, blast %.1f\n",
        stage.piloting,
        stage.piloting ? stage.aircraft[stage.piloting - 1].position.y : 0.0f,
        stage.piloting ? stage.aircraft[stage.piloting - 1].speed * 3.6f : 0.0f,
        stage.blast);
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
  if (getenv("SNAPSHOT_ORBIT")) {
    /* Look at the car from `degrees` round from behind, at a distance. */
    const rigid_body_t* body = physics_body(stage.physics, stage.vehicle.body);
    float angle = (float)atof(getenv("SNAPSHOT_ORBIT")) * (float)M_PI / 180.0f;
    float distance = getenv("SNAPSHOT_ORBIT_DISTANCE")
                         ? (float)atof(getenv("SNAPSHOT_ORBIT_DISTANCE"))
                         : 12.0f;
    vec3_t behind =
        quat_rotate(body->rotation, vec3(sinf(angle), 0.25f, cosf(angle)));
    stage.camera.position =
        vec3_add(body->position, vec3_scale(behind, distance));
    stage.camera.look_target = body->position;
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
