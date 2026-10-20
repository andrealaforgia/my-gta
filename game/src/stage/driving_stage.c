#include "driving_stage.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "city_scene.h"
#include "log.h"

#define PHYSICS_STEP (1.0 / 120.0)
#define MAX_CATCH_UP 0.25
#define IMPACT_THRESHOLD 400.0f
#define IMPACT_FULL 6000.0f

bool driving_stage_init(driving_stage_t* stage, window_t* window,
                        renderer_t* renderer, audio_device_t* audio,
                        stage_options_t options) {
  memset(stage, 0, sizeof(*stage));
  stage->window = window;
  stage->renderer = renderer;
  stage->audio = audio;
  stage->show_debug = options.show_debug;
  stage->physics = physics_world_create();
  if (!stage->physics) return false;
  stage->world = world_create();
  stage->textures = texture_set_upload(renderer);
  scene_context_t context = {renderer, stage->physics, &stage->world,
                             &stage->textures};
  bool built =
      options.scene == SCENE_CITY
          ? scene_build_city(&stage->scene, context, options.city_package)
          : scene_build_test_track(&stage->scene, context);
  if (!built) return false;

  vehicle_params_t params;
  const char* vehicle_file = options.vehicle_file
                                 ? options.vehicle_file
                                 : "assets/vehicles/hatchback.cfg";
  if (!vehicle_params_load(&params, vehicle_file)) return false;
  if (!vehicle_create(&stage->vehicle, stage->physics, &params,
                      stage->scene.spawn_position, stage->scene.spawn_rotation))
    return false;
  stage->car_visual = car_visual_create(renderer, &params);
  stage->camera = chase_camera_create(chase_camera_default_params());
  chase_camera_reset(&stage->camera, transform_at(stage->scene.spawn_position,
                                                  stage->scene.spawn_rotation));
  stage->clock = frame_clock_create(PHYSICS_STEP, MAX_CATCH_UP);
  if (audio && options.muted) audio_device_toggle_mute(audio);
  LOG_INFO("stage", "%s loaded, car '%s' at (%.1f, %.1f, %.1f)",
           stage->scene.name, params.name, stage->scene.spawn_position.x,
           stage->scene.spawn_position.y, stage->scene.spawn_position.z);
  return true;
}

void driving_stage_destroy(driving_stage_t* stage) {
  if (stage->renderer) {
    car_visual_destroy(&stage->car_visual, stage->renderer);
    scene_release(&stage->scene, stage->renderer);
  }
  world_destroy(&stage->world);
  physics_world_destroy(stage->physics);
  stage->physics = NULL;
}

static void send_audio_state(driving_stage_t* stage) {
  if (!stage->audio) return;
  const vehicle_t* car = &stage->vehicle;
  audio_state_t state = audio_state_default();
  state.engine_running = true;
  state.rpm = vehicle_rpm(car);
  state.throttle =
      car->gear == VEHICLE_GEAR_REVERSE ? car->brake : car->throttle;
  state.skid = vehicle_skid_intensity(car);
  state.speed_mps = fabsf(vehicle_forward_speed(car));
  const rigid_body_t* body = physics_body(stage->physics, car->body);
  state.listener_distance =
      body ? vec3_distance(stage->camera.position, body->position) : 5.0f;
  audio_device_set_state(stage->audio, state);
  if (car->impact_impulse > IMPACT_THRESHOLD)
    audio_device_trigger_impact(stage->audio,
                                car->impact_impulse / IMPACT_FULL);
}

void driving_stage_step(driving_stage_t* stage, driving_input_t input,
                        float dt) {
  vehicle_update(&stage->vehicle, stage->physics, input, dt);
  physics_world_step(stage->physics, dt);
  stage->simulated_seconds += dt;
}

static transform_t car_transform(driving_stage_t* stage, float alpha) {
  const rigid_body_t* body = physics_body(stage->physics, stage->vehicle.body);
  return body ? physics_body_interpolated(body, alpha) : transform_identity();
}

void driving_stage_advance(driving_stage_t* stage, driving_input_t input,
                           double frame_seconds) {
  double started = platform_seconds();
  int steps =
      frame_clock_advance(&stage->clock, stage->paused ? 0.0 : frame_seconds);
  for (int i = 0; i < steps; i++)
    driving_stage_step(stage, input, (float)PHYSICS_STEP);
  stage->physics_ms = (platform_seconds() - started) * 1000.0;
  send_audio_state(stage);
  const rigid_body_t* body = physics_body(stage->physics, stage->vehicle.body);
  if (body && !stage->paused)
    chase_camera_update(&stage->camera,
                        car_transform(stage, (float)stage->clock.alpha),
                        body->velocity, stage->physics, (float)frame_seconds);
}

static void draw_debug(driving_stage_t* stage, transform_t chassis) {
  renderer_t* r = stage->renderer;
  const vehicle_t* car = &stage->vehicle;
  for (size_t i = 0; i < physics_collider_count(stage->physics); i++) {
    const static_collider_t* collider = physics_collider_at(stage->physics, i);
    if (!collider->used) continue;
    if (collider->shape == COLLIDER_SHAPE_BOX) {
      renderer_draw_box_outline(r, transform_to_matrix(collider->transform),
                                collider->half_extents, vec3(0.2f, 1.0f, 0.4f));
    } else if (vec3_distance(aabb_center(collider->bounds), chassis.position) <
               150.0f) {
      renderer_draw_box_outline(
          r, mat4_translation(aabb_center(collider->bounds)),
          aabb_half_extents(collider->bounds), vec3(0.2f, 0.6f, 1.0f));
    }
  }
  const rigid_body_t* body = physics_body(stage->physics, car->body);
  renderer_draw_box_outline(
      r, transform_to_matrix(transform_at(body->position, body->rotation)),
      body->half_extents, vec3(1, 1, 0));
  renderer_draw_line(r, body->position,
                     vec3_add(body->position, body->velocity), vec3(1, 0, 1));
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
    const wheel_state_t* wheel = &car->wheels[i];
    vec3_t attach = vec3_add(body->position,
                             quat_rotate(body->rotation, wheel->attach_local));
    vec3_t color = wheel->skidding   ? vec3(1, 0.2f, 0.2f)
                   : wheel->grounded ? vec3(0.2f, 1, 0.2f)
                                     : vec3(0.6f, 0.6f, 0.6f);
    renderer_draw_line(
        r, attach, wheel->grounded ? wheel->contact_point : wheel->hub_position,
        color);
    if (wheel->grounded)
      renderer_draw_line(
          r, wheel->contact_point,
          vec3_add_scaled(wheel->contact_point, wheel->contact_normal,
                          wheel->load / 10000.0f),
          vec3(1, 1, 1));
  }
  for (size_t i = 0; i < physics_contact_count(stage->physics); i++) {
    const contact_event_t* contact = physics_contact_at(stage->physics, i);
    renderer_draw_line(r, contact->point,
                       vec3_add_scaled(contact->point, contact->normal, 0.5f),
                       vec3(1, 0.5f, 0));
  }
}

static void draw_hud(driving_stage_t* stage) {
  renderer_t* r = stage->renderer;
  const vehicle_t* car = &stage->vehicle;
  char line[128];
  float scale = (float)stage->window->drawable_width / 640.0f;
  if (scale < 2) scale = 2;
  float x = 16 * scale / 2, y = 16 * scale / 2;
  snprintf(line, sizeof(line), "%3.0f KM/H", fabsf(vehicle_speed_kmh(car)));
  renderer_draw_text(r, x, y, scale * 2.5f, vec3(1, 1, 1), line);
  y += 26 * scale;
  int gear = vehicle_gear(car);
  char gear_text[8];
  if (gear == VEHICLE_GEAR_REVERSE)
    snprintf(gear_text, sizeof(gear_text), "R");
  else
    snprintf(gear_text, sizeof(gear_text), "%d", gear);
  int bars = (int)(vehicle_rpm(car) / car->params.engine_max_rpm * 20);
  char bar[24];
  for (int i = 0; i < 20; i++) bar[i] = i < bars ? '#' : '.';
  bar[20] = '\0';
  snprintf(line, sizeof(line), "GEAR %s  RPM %4.0f %s", gear_text,
           vehicle_rpm(car), bar);
  renderer_draw_text(r, x, y, scale * 1.4f, vec3(1, 0.9f, 0.6f), line);
  y += 14 * scale;
  snprintf(line, sizeof(line), "THR %3.0f%% BRK %3.0f%% STR %+4.2f %s",
           car->throttle * 100, car->brake * 100, car->steering,
           car->handbrake ? "HANDBRAKE" : "");
  renderer_draw_text(r, x, y, scale * 1.2f, vec3(0.8f, 0.8f, 0.8f), line);
  y += 12 * scale;
  render_stats_t stats = renderer_stats(r);
  snprintf(line, sizeof(line),
           "%4.0f FPS %5.2f MS FRAME %5.2f MS PHYSICS %d DRAWS %d TRIS",
           stage->fps, stage->frame_ms, stage->physics_ms, stats.draw_calls,
           stats.triangles);
  renderer_draw_text(r, x, y, scale * 1.0f, vec3(0.7f, 0.9f, 0.7f), line);
  if (stage->paused) {
    renderer_draw_text(r, (float)stage->window->drawable_width / 2 - 30 * scale,
                       (float)stage->window->drawable_height / 2, scale * 3,
                       vec3(1, 1, 0.3f), "PAUSED");
  }
  if (stage->show_debug) {
    y += 12 * scale;
    for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
      const wheel_state_t* w = &car->wheels[i];
      snprintf(line, sizeof(line),
               "W%d %s COMP %.3f LOAD %5.0f SLIP %+.2f GRIP %.2f%s", i,
               w->grounded ? "GND" : "AIR", w->compression, w->load,
               w->slip_angle, w->grip_use, w->skidding ? " SKID" : "");
      renderer_draw_text(r, x, y, scale, vec3(0.9f, 0.9f, 0.5f), line);
      y += 10 * scale;
    }
  }
  float bottom = (float)stage->window->drawable_height - 12 * scale;
  renderer_draw_text(r, x, bottom, scale * 0.9f, vec3(0.6f, 0.6f, 0.6f),
                     "ARROWS/WASD DRIVE  SPACE HANDBRAKE  R RESET  P PAUSE  F1 "
                     "DEBUG  M MUTE  F11 FULLSCREEN  ESC QUIT");
}

void driving_stage_render(driving_stage_t* stage) {
  renderer_t* r = stage->renderer;
  transform_t chassis = car_transform(stage, (float)stage->clock.alpha);
  renderer_begin_frame(r, chase_camera_render(&stage->camera),
                       stage->scene.environment);
  stage->last_drawn = world_render(&stage->world, r);
  car_visual_draw(&stage->car_visual, r, &stage->vehicle, chassis);
  if (stage->show_debug) draw_debug(stage, chassis);
  draw_hud(stage);
  renderer_end_frame(r);
}

static void handle_events(driving_stage_t* stage, window_events_t events) {
  if (events.quit) stage->quit = true;
  if (events.resized)
    renderer_resize(stage->renderer, stage->window->drawable_width,
                    stage->window->drawable_height);
  if (events.reset_vehicle) {
    vehicle_reset(&stage->vehicle, stage->physics);
    chase_camera_reset(&stage->camera,
                       transform_at(stage->vehicle.spawn_position,
                                    stage->vehicle.spawn_rotation));
  }
  if (events.toggle_pause) stage->paused = !stage->paused;
  if (events.toggle_debug) stage->show_debug = !stage->show_debug;
  if (events.toggle_fullscreen) {
    window_toggle_fullscreen(stage->window);
    renderer_resize(stage->renderer, stage->window->drawable_width,
                    stage->window->drawable_height);
  }
  if (events.toggle_mute && stage->audio)
    audio_device_toggle_mute(stage->audio);
  if (events.toggle_camera)
    chase_camera_reset(&stage->camera, car_transform(stage, 1.0f));
}

void driving_stage_run(driving_stage_t* stage) {
  double previous = platform_seconds();
  while (!stage->quit) {
    double now = platform_seconds();
    double frame_seconds = now - previous;
    previous = now;
    handle_events(stage, window_poll(stage->window));
    driving_input_t input = window_driving_input(stage->window);
    driving_stage_advance(stage, input, frame_seconds);
    driving_stage_render(stage);
    window_swap(stage->window);
    stage->frame_ms = (platform_seconds() - now) * 1000.0;
    stage->fps_accumulator += frame_seconds;
    stage->fps_frames++;
    if (stage->fps_accumulator >= 0.5) {
      stage->fps = stage->fps_frames / stage->fps_accumulator;
      stage->fps_accumulator = 0;
      stage->fps_frames = 0;
    }
  }
}
