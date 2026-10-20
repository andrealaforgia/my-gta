#include "driving_stage.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "city_scene.h"
#include "daylight.h"
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
  stage->hour = options.start_hour;
  stage->hour_seconds = options.hour_seconds;
  if (stage->scene.map.size > 0)
    stage->map_texture =
        renderer_upload_texture(renderer, stage->scene.map.size,
                                stage->scene.map.size, stage->scene.map.pixels);
  if (audio && options.muted) audio_device_toggle_mute(audio);
  LOG_INFO("stage", "%s loaded, car '%s' at (%.1f, %.1f, %.1f)",
           stage->scene.name, params.name, stage->scene.spawn_position.x,
           stage->scene.spawn_position.y, stage->scene.spawn_position.z);
  return true;
}

void driving_stage_destroy(driving_stage_t* stage) {
  if (stage->renderer) {
    car_visual_destroy(&stage->car_visual, stage->renderer);
    if (stage->map_texture.id)
      renderer_release_texture(stage->renderer, stage->map_texture);
    scene_release(&stage->scene, stage->renderer);
  }
  world_destroy(&stage->world);
  physics_world_destroy(stage->physics);
  stage->physics = NULL;
}

audio_state_t driving_stage_audio_state(const driving_stage_t* stage) {
  const vehicle_t* car = &stage->vehicle;
  audio_state_t state = audio_state_default();
  /* Paused, or looking at the map: the world holds still and so does its
   * sound. A frozen engine note would only hum. */
  if (stage->paused || stage->map_open) {
    state.engine_running = false;
    return state;
  }
  state.engine_running = true;
  state.rpm = vehicle_rpm(car);
  state.throttle =
      car->gear == VEHICLE_GEAR_REVERSE ? car->brake : car->throttle;
  state.skid = vehicle_skid_intensity(car);
  state.speed_mps = fabsf(vehicle_forward_speed(car));
  const rigid_body_t* body = physics_body(stage->physics, car->body);
  state.listener_distance =
      body ? vec3_distance(stage->camera.position, body->position) : 5.0f;
  return state;
}

static void send_audio_state(driving_stage_t* stage) {
  if (!stage->audio) return;
  audio_device_set_state(stage->audio, driving_stage_audio_state(stage));
  if (!stage->paused && !stage->map_open &&
      stage->vehicle.impact_impulse > IMPACT_THRESHOLD)
    audio_device_trigger_impact(stage->audio,
                                stage->vehicle.impact_impulse / IMPACT_FULL);
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
  int steps = frame_clock_advance(
      &stage->clock, stage->paused || stage->map_open ? 0.0 : frame_seconds);
  for (int i = 0; i < steps; i++)
    driving_stage_step(stage, input, (float)PHYSICS_STEP);
  stage->physics_ms = (platform_seconds() - started) * 1000.0;
  if (!stage->paused && !stage->map_open && stage->hour_seconds > 0) {
    stage->hour += (float)frame_seconds / stage->hour_seconds;
    if (stage->hour >= 24.0f) stage->hour -= 24.0f;
  }
  send_audio_state(stage);
  const rigid_body_t* body = physics_body(stage->physics, stage->vehicle.body);
  if (body && !stage->paused && !stage->map_open)
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

/* M: the city as a dark translucent panel, north up, with the car on it. */
static void draw_map(driving_stage_t* stage) {
  renderer_t* r = stage->renderer;
  float width = (float)stage->window->drawable_width;
  float height = (float)stage->window->drawable_height;
  float scale = width / 640.0f;
  if (scale < 2) scale = 2;
  texture_handle_t none = {0};
  renderer_draw_screen_image(r, none, 0, 0, width, height,
                             vec3(0.16f, 0.16f, 0.17f), 0.78f);
  const city_map_t* map = &stage->scene.map;
  if (map->size == 0) {
    renderer_draw_text(r, width / 2 - 70 * scale, height / 2, scale * 2,
                       vec3(0.9f, 0.9f, 0.9f), "NO MAP HERE");
    return;
  }
  float side = fminf(width, height) * 0.86f;
  float left = (width - side) * 0.5f, top = (height - side) * 0.5f;
  renderer_draw_screen_image(r, stage->map_texture, left, top, side, side,
                             vec3(1, 1, 1), 0.92f);
  /* The car: a dot with a nose pointing the way it faces. */
  transform_t chassis = car_transform(stage, 1.0f);
  vec3_t forward = quat_rotate(chassis.rotation, vec3(0, 0, -1));
  float pixels_per_metre = side / map->extent;
  float cx = left + (chassis.position.x - map->min_x) * pixels_per_metre;
  float cy = top + (chassis.position.z - map->min_z) * pixels_per_metre;
  float length = sqrtf(forward.x * forward.x + forward.z * forward.z);
  float dx = length > 1e-4f ? forward.x / length : 0;
  float dz = length > 1e-4f ? forward.z / length : -1;
  float arm = 7 * scale;
  vec3_t red = vec3(1.0f, 0.15f, 0.1f), white = vec3(1, 1, 1);
  renderer_draw_screen_line(r, cx - dx * arm, cy - dz * arm,
                            cx + dx * arm * 1.6f, cy + dz * arm * 1.6f,
                            3 * scale, white);
  renderer_draw_screen_line(r, cx - dx * arm, cy - dz * arm,
                            cx + dx * arm * 1.6f, cy + dz * arm * 1.6f,
                            1.6f * scale, red);
  renderer_draw_screen_line(r, cx + dx * arm * 1.6f, cy + dz * arm * 1.6f,
                            cx + dx * arm * 0.6f - dz * arm * 0.7f,
                            cy + dz * arm * 0.6f + dx * arm * 0.7f,
                            1.6f * scale, red);
  renderer_draw_screen_line(r, cx + dx * arm * 1.6f, cy + dz * arm * 1.6f,
                            cx + dx * arm * 0.6f + dz * arm * 0.7f,
                            cy + dz * arm * 0.6f - dx * arm * 0.7f,
                            1.6f * scale, red);
  /* Street names, longest streets first, skipping any that would land on
   * one already placed. */
  float text_scale = scale * 0.55f;
  float placed[CITY_PACKAGE_MAX_LABELS][4];
  size_t placed_count = 0;
  for (size_t i = 0; i < stage->scene.label_count && i < 18; i++) {
    const city_label_t* label = &stage->scene.labels[i];
    float w = (float)strlen(label->text) * 6 * text_scale, h = 8 * text_scale;
    float lx = left + (label->x - map->min_x) * pixels_per_metre - w * 0.5f;
    float ly = top + (label->z - map->min_z) * pixels_per_metre - h * 0.5f;
    bool clash = false;
    for (size_t k = 0; k < placed_count; k++)
      if (lx < placed[k][2] && lx + w > placed[k][0] && ly < placed[k][3] &&
          ly + h > placed[k][1])
        clash = true;
    if (clash) continue;
    placed[placed_count][0] = lx, placed[placed_count][1] = ly;
    placed[placed_count][2] = lx + w, placed[placed_count][3] = ly + h;
    placed_count++;
    renderer_draw_text(r, lx + text_scale * 0.5f, ly + text_scale * 0.5f,
                       text_scale, vec3(0.1f, 0.1f, 0.12f), label->text);
    renderer_draw_text(r, lx, ly, text_scale, vec3(1.0f, 0.97f, 0.8f),
                       label->text);
  }
  /* Title, north marker and a 100 m scale bar. */
  vec3_t light = vec3(0.92f, 0.92f, 0.92f);
  renderer_draw_text(r, left, top - 14 * scale, scale * 1.6f, light,
                     stage->scene.name);
  renderer_draw_text(r, left + side - 150 * scale, top - 14 * scale,
                     scale * 1.2f, vec3(0.7f, 0.7f, 0.7f), "M CLOSE");
  renderer_draw_text(r, left + side * 0.5f - 3 * scale, top + 4 * scale,
                     scale * 1.8f, vec3(1, 0.9f, 0.4f), "N");
  float bar = 100.0f * pixels_per_metre;
  renderer_draw_screen_line(r, left + 8 * scale, top + side - 10 * scale,
                            left + 8 * scale + bar, top + side - 10 * scale,
                            2 * scale, white);
  renderer_draw_text(r, left + 8 * scale, top + side - 26 * scale, scale, white,
                     "100 M");
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
    const rigid_body_t* body = physics_body(stage->physics, car->body);
    ray_t down = {body->position, vec3(0, -1, 0)};
    ray_cast_result_t under;
    if (physics_ray_cast(stage->physics, down, 5.0f, COLLISION_LAYER_ALL,
                         &under)) {
      snprintf(line, sizeof(line),
               "SURFACE LAYER %d FRICTION %.2f SOURCE %llu AT %.1f %.1f %.1f",
               (int)under.layer, under.friction,
               (unsigned long long)under.source_id, body->position.x,
               body->position.y, body->position.z);
    } else {
      snprintf(line, sizeof(line), "SURFACE NONE AT %.1f %.1f %.1f",
               body->position.x, body->position.y, body->position.z);
    }
    renderer_draw_text(r, x, y, scale, vec3(0.6f, 0.9f, 1.0f), line);
    y += 10 * scale;
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
                     "DEBUG  M MAP  N MUTE  F11 FULLSCREEN  ESC QUIT");
}

void driving_stage_render(driving_stage_t* stage) {
  renderer_t* r = stage->renderer;
  transform_t chassis = car_transform(stage, (float)stage->clock.alpha);
  float night = daylight_lamp_glow(stage->hour);
  render_environment_t environment = daylight_environment(stage->hour);
  environment.fog_start = stage->scene.environment.fog_start;
  environment.fog_end = stage->scene.environment.fog_end;
  environment.shadow_radius = stage->scene.environment.shadow_radius;
  /* The sun's shadows fade with it; the moon casts none. */
  if (night > 0.7f) environment.shadow_radius = 0;
  /* Headlights: a cone from the nose along the car's heading. */
  vec3_t forward = quat_rotate(chassis.rotation, vec3(0, 0, -1));
  environment.spot_position = vec3_add(
      chassis.position, vec3_add(vec3_scale(forward, 2.0f), vec3(0, 0.1f, 0)));
  environment.spot_direction =
      vec3_normalize(vec3_add(forward, vec3(0, -0.12f, 0)));
  environment.spot_intensity = 2.2f * night;
  for (size_t i = 0; i < stage->world.capacity; i++) {
    entity_t* entity = &stage->world.entities[i];
    if (entity->used && strncmp(entity->label, "lamp_light", 10) == 0)
      entity->material.emissive = 2.4f * night;
  }
  renderer_begin_frame(r, chase_camera_render(&stage->camera), environment);
  stage->last_drawn = world_render(&stage->world, r);
  car_visual_draw(&stage->car_visual, r, &stage->vehicle, chassis, night);
  if (stage->show_debug) draw_debug(stage, chassis);
  if (stage->map_open)
    draw_map(stage);
  else
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
  if (events.toggle_map) stage->map_open = !stage->map_open;
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
