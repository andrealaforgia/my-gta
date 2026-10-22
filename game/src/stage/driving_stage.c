#include "driving_stage.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "city_scene.h"
#include "daylight.h"
#include "log.h"
#include "poles.h"
#include "street_name.h"

#define PHYSICS_STEP (1.0 / 120.0)
#define MAX_CATCH_UP 0.25
#define IMPACT_THRESHOLD 400.0f
#define IMPACT_FULL 6000.0f
#define CRIME_IMPACT 3600.0f /* a crash this hard is a crime */
#define GET_IN_REACH 3.2f    /* metres from a car to its door */

/* The chase camera sits further back and higher behind a long vehicle. */
static chase_camera_params_t chase_camera_params_for(
    const vehicle_params_t* params) {
  chase_camera_params_t camera = chase_camera_default_params();
  float extra = fmaxf(0.0f, params->chassis_half_extents.z - 2.4f);
  camera.distance += extra * 1.7f;
  camera.height += extra * 0.45f;
  camera.look_height += extra * 0.15f;
  return camera;
}

bool driving_stage_init(driving_stage_t* stage, window_t* window,
                        renderer_t* renderer, audio_device_t* audio,
                        stage_options_t options) {
  memset(stage, 0, sizeof(*stage));
  stage->window = window;
  stage->renderer = renderer;
  stage->audio = audio;
  stage->show_debug = options.show_debug;
  stage->has_menu = options.has_menu;
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
  stage->camera = chase_camera_create(chase_camera_params_for(&params));
  chase_camera_reset(&stage->camera, transform_at(stage->scene.spawn_position,
                                                  stage->scene.spawn_rotation));
  stage->clock = frame_clock_create(PHYSICS_STEP, MAX_CATCH_UP);
  if (options.scene == SCENE_CITY && fleet_create(&stage->fleet, renderer)) {
    stage->has_fleet = true;
    parked_cars_create(&stage->parked, stage->scene.parked,
                       stage->scene.parked_count, stage->physics,
                       &stage->fleet);
    if (options.traffic_cars > 0 && stage->scene.route_count > 0)
      stage->traffic = traffic_create(
          stage->scene.routes, stage->scene.route_count, stage->physics,
          &stage->fleet, options.traffic_cars, 2026);
  }
  if (stage->has_fleet && options.pedestrians > 0 &&
      stage->scene.route_count > 0) {
    stage->people =
        pedestrians_create(stage->scene.routes, stage->scene.route_count,
                           stage->physics, options.pedestrians, 2027);
    if (stage->people) {
      pedestrians_set_crossings(stage->people, stage->scene.crossings,
                                stage->scene.crossing_count);
    }
  }
  if (stage->has_fleet) {
    stage->people_meshes = pedestrian_meshes_create(renderer);
    stage->has_figures = true;
    for (int outfit = 0; outfit < 200; outfit++)
      if (pedestrian_kind(outfit) == PERSON_ADULT) {
        stage->walker_outfit = outfit;
        break;
      }
  }
  if (stage->traffic)
    traffic_set_bus_stops(stage->traffic, stage->scene.bus_stops,
                          stage->scene.bus_stop_count);
  if (stage->traffic)
    traffic_set_signals(stage->traffic, stage->scene.signals,
                        stage->scene.signal_count);
  if (options.scene == SCENE_CITY && stage->scene.route_count > 0) {
    mission_init(&stage->mission, 7);
    wanted_init(&stage->wanted);
    stage->nav = nav_create(stage->scene.routes, stage->scene.route_count);
    if (stage->traffic) traffic_set_nav(stage->traffic, stage->nav);
    stage->mission_markers = mission_markers_create(renderer);
    stage->missions = true;
  }
  skid_marks_reset(&stage->skid_marks);
  stage->skid_strip = skid_mark_mesh_create(renderer);
  stage->rain = options.rain;
  stage->rain_target = options.rain;
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
    renderer_release_mesh(stage->renderer, stage->skid_strip);
    if (stage->missions)
      mission_markers_destroy(&stage->mission_markers, stage->renderer);
    nav_destroy(stage->nav);
    traffic_destroy(stage->traffic, stage->physics);
    if (stage->has_figures)
      pedestrian_meshes_destroy(&stage->people_meshes, stage->renderer);
    pedestrians_destroy(stage->people);
    parked_cars_destroy(&stage->parked, stage->physics);
    if (stage->has_fleet) fleet_destroy(&stage->fleet, stage->renderer);
    if (stage->map_texture.id)
      renderer_release_texture(stage->renderer, stage->map_texture);
    scene_release(&stage->scene, stage->renderer);
  }
  world_destroy(&stage->world);
  physics_world_destroy(stage->physics);
  stage->physics = NULL;
}

/* Where the player is: the car, or the person who has got out of it. */
static vec3_t player_position(const driving_stage_t* stage) {
  if (stage->on_foot) return stage->walker.position;
  return physics_body(stage->physics, stage->vehicle.body)->position;
}

static vec3_t player_velocity(const driving_stage_t* stage) {
  if (stage->on_foot)
    return vec3_scale(walker_forward(&stage->walker), stage->walker.speed);
  return physics_body(stage->physics, stage->vehicle.body)->velocity;
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
  state.engine_running = !stage->on_foot;
  state.horn = stage->horn && !stage->on_foot;
  state.rain = stage->rain;
  state.city = 1.0f * (1.0f - 0.7f * stage->rain);
  state.night = daylight_lamp_glow(stage->hour);
  if (stage->traffic) {
    vec3_t me = player_position(stage);
    float d = traffic_nearest_distance(stage->traffic, me);
    state.traffic = d < 30.0f ? 1.0f - d / 30.0f : 0.0f;
    float siren = traffic_nearest_siren(stage->traffic, me);
    state.siren = siren < 100.0f ? 1.0f - siren / 100.0f : 0.0f;
    float honk = traffic_nearest_honk(stage->traffic, me);
    state.other_horn = honk < 60.0f ? 1.0f - honk / 60.0f : 0.0f;
  }
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
  /* The impact sound goes by how hard the hit was for this vehicle. */
  float felt = stage->vehicle.impact_impulse /
               fmaxf(stage->vehicle.params.mass / 1200.0f, 1.0f);
  if (!stage->paused && !stage->map_open && felt > IMPACT_THRESHOLD)
    audio_device_trigger_impact(stage->audio, felt / IMPACT_FULL);
}

static void say(driving_stage_t* stage, const char* text) {
  snprintf(stage->notice, sizeof(stage->notice), "%s", text);
  stage->notice_time = 3.0f;
}

static void back_to_spawn(driving_stage_t* stage) {
  if (stage->on_foot) {
    /* Back in the car, which is back at the start. */
    stage->on_foot = false;
    stage->camera.params = stage->car_camera_params;
  }
  vehicle_reset(&stage->vehicle, stage->physics);
  chase_camera_reset(&stage->camera,
                     transform_at(stage->vehicle.spawn_position,
                                  stage->vehicle.spawn_rotation));
}

/* The law's side of the drive: crashes are crimes, police cars come, and the
 * chase ends in being caught or getting away. */
static void step_the_law(driving_stage_t* stage, bool pole_down, float dt) {
  if (stage->notice_time > 0) stage->notice_time -= dt;
  if (!stage->missions || !stage->traffic) return;
  float felt = stage->vehicle.impact_impulse /
               fmaxf(stage->vehicle.params.mass / 1200.0f, 1.0f);
  wanted_event_t event = WANTED_NOTHING;
  if (!stage->on_foot && (felt > CRIME_IMPACT || pole_down))
    event = wanted_crime(&stage->wanted);
  if (event == WANTED_STARTED) say(stage, "WANTED");
  traffic_set_pursuit(stage->traffic, wanted_pursuers(&stage->wanted));
  vec3_t me = player_position(stage);
  float speed = vec3_length(player_velocity(stage));
  event = wanted_step(&stage->wanted,
                      traffic_nearest_pursuer(stage->traffic, me), speed, dt);
  if (event == WANTED_BUSTED) {
    int fine = stage->mission.cash / 4;
    stage->mission.cash -= fine;
    char text[40];
    snprintf(text, sizeof(text), "BUSTED  -$%d", fine);
    say(stage, text);
    back_to_spawn(stage);
  } else if (event == WANTED_ESCAPED) {
    say(stage, "LOST THEM");
  }
}

void driving_stage_step(driving_stage_t* stage, driving_input_t input,
                        float dt) {
  /* On foot the car stands with its handbrake on and the input walks. */
  driving_input_t parked_car_input = {0.0f, 0.0f, 0.0f, true};
  vehicle_update(&stage->vehicle, stage->physics,
                 stage->on_foot ? parked_car_input : input, dt);
  if (stage->on_foot) {
    walker_input_t walking = {input.throttle - input.brake, input.steering,
                              input.handbrake};
    walker_step(&stage->walker, stage->physics, walking, dt);
    const rigid_body_t* car = physics_body(stage->physics, stage->vehicle.body);
    vec3_t nose = quat_rotate(car->rotation, vec3(0, 0, -1));
    walker_push_out_of_box(&stage->walker, car->position,
                           atan2f(-nose.x, -nose.z),
                           stage->vehicle.params.chassis_half_extents);
  }
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
    const wheel_state_t* wheel = &stage->vehicle.wheels[i];
    skid_marks_wheel(&stage->skid_marks, i, wheel->contact_point,
                     wheel->grounded && wheel->skidding);
  }
  stage->signal_clock += dt;
  if (stage->traffic) {
    traffic_set_signal_time(stage->traffic, stage->signal_clock);
    vec3_t where = player_position(stage);
    traffic_set_focus(stage->traffic, where);
    traffic_step(stage->traffic, stage->physics, where, player_velocity(stage),
                 dt);
  }
  if (stage->people) {
    pedestrians_set_focus(stage->people, player_position(stage));
    pedestrians_step(stage->people, player_position(stage), dt);
  }
  if (stage->traffic) {
    /* People in the road, and the car the player has left standing: cars
     * stop for them. */
    size_t standing =
        stage->people ? pedestrians_crossing(stage->people, stage->walkers, 63)
                      : 0;
    if (stage->on_foot)
      stage->walkers[standing++] =
          physics_body(stage->physics, stage->vehicle.body)->position;
    traffic_set_walkers(stage->traffic, stage->walkers, standing);
  }
  rigid_body_t* chassis_body =
      physics_body(stage->physics, stage->vehicle.body);
  vec3_t velocity_before = chassis_body->velocity;
  physics_world_step(stage->physics, dt);
  /* A pole the car hits at speed goes over, and gives way rather than
   * stopping the car dead. */
  bool pole_down =
      poles_topple_struck(&stage->scene, stage->physics, &stage->world,
                          stage->vehicle.body, velocity_before) > 0;
  if (pole_down) {
    chassis_body = physics_body(stage->physics, stage->vehicle.body);
    vec3_t after = chassis_body->velocity;
    chassis_body->velocity =
        vec3(after.x + (velocity_before.x - after.x) * 0.6f, after.y,
             after.z + (velocity_before.z - after.z) * 0.6f);
  }
  poles_step(&stage->scene, &stage->world, dt);
  step_the_law(stage, pole_down, dt);
  if (stage->missions) {
    vec3_t at = player_position(stage);
    /* The way to the destination, worked out again every second or two. */
    stage->satnav_clock += dt;
    if (!stage->mission.active) {
      stage->satnav_count = 0;
    } else if (stage->nav &&
               (stage->satnav_clock > 1.5f || stage->satnav_count == 0)) {
      stage->satnav_clock = 0;
      stage->satnav_count =
          nav_path(stage->nav, at.x, at.z, stage->mission.x, stage->mission.z,
                   stage->satnav, 1024, NULL);
    }
    mission_event_t started =
        mission_step(&stage->mission, stage->scene.routes,
                     stage->scene.route_count, at.x, at.z, dt);
    /* A destination no road leads to is no use: ask for another. */
    if (started == MISSION_STARTED && stage->nav &&
        nav_path(stage->nav, at.x, at.z, stage->mission.x, stage->mission.z,
                 stage->satnav, 1024, NULL) == 0) {
      stage->mission.active = false;
      stage->mission.pause = 0.5f;
      started = MISSION_NOTHING;
    }
    if (started == MISSION_STARTED) {
      ray_t down = {vec3(stage->mission.x, 300.0f, stage->mission.z),
                    vec3(0, -1, 0)};
      ray_cast_result_t hit;
      stage->mission_ground = physics_ray_cast(stage->physics, down, 600.0f,
                                               COLLISION_LAYER_GROUND, &hit)
                                  ? hit.hit.point.y
                                  : 0.0f;
    }
  }
  /* The name of the street under the car, looked up four times a second. */
  stage->street_clock += dt;
  if (stage->street_clock >= 0.25f && stage->scene.route_count > 0) {
    stage->street_clock = 0;
    vec3_t at = player_position(stage);
    street_name_at(stage->scene.routes, stage->scene.route_count, at.x, at.z,
                   stage->street, sizeof(stage->street));
  }
  /* A big city is streamed: what is near the car is in the world. */
  scene_stream(&stage->scene, player_position(stage), 1);
  /* A hard hit jolts the view, scaled to the vehicle as the sound is. */
  stage->shake = fmaxf(stage->shake - 2.5f * dt, 0.0f);
  float felt = stage->vehicle.impact_impulse /
               fmaxf(stage->vehicle.params.mass / 1200.0f, 1.0f);
  if (felt > IMPACT_THRESHOLD)
    stage->shake = fmaxf(stage->shake, fminf(felt / IMPACT_FULL, 1.0f));
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
  if (!stage->paused && !stage->map_open) {
    float step =
        (float)frame_seconds / 6.0f; /* a shower comes on in six seconds */
    if (stage->rain < stage->rain_target)
      stage->rain = fminf(stage->rain_target, stage->rain + step);
    else
      stage->rain = fmaxf(stage->rain_target, stage->rain - step);
    stage->rain_clock += (float)frame_seconds;
  }
  if (!stage->paused && !stage->map_open && stage->hour_seconds > 0) {
    float before = stage->hour;
    stage->hour += (float)frame_seconds / stage->hour_seconds;
    if (stage->hour >= 24.0f) stage->hour -= 24.0f;
    int strikes = daylight_bell_strikes(before, stage->hour);
    if (strikes > 0 && stage->audio)
      audio_device_trigger_bell(stage->audio, strikes);
  }
  send_audio_state(stage);
  const rigid_body_t* body = physics_body(stage->physics, stage->vehicle.body);
  if (body && !stage->paused && !stage->map_open) {
    if (stage->on_foot) {
      /* Behind the person, at about shoulder height. */
      transform_t head =
          transform_at(vec3_add(stage->walker.position, vec3(0, 0.9f, 0)),
                       quat_from_yaw(stage->walker.yaw));
      chase_camera_update(&stage->camera, head, player_velocity(stage),
                          stage->physics, (float)frame_seconds);
    } else {
      chase_camera_update(&stage->camera,
                          car_transform(stage, (float)stage->clock.alpha),
                          body->velocity, stage->physics, (float)frame_seconds);
    }
  }
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
  if (stage->missions) {
    mission_route_on_map(r, map, stage->satnav, stage->satnav_count, left, top,
                         side, scale);
    mission_map_mark(&stage->mission, r, map, left, top, side, scale,
                     stage->rain_clock);
  }
}

/* The name of the street the car is on, at the top in the middle on a dark
 * plate, so it is easy to read against anything. */
static void draw_street_name(driving_stage_t* stage) {
  if (stage->street[0] == '\0') return;
  renderer_t* r = stage->renderer;
  float scale = (float)stage->window->drawable_width / 640.0f;
  if (scale < 2) scale = 2;
  float text_scale = scale * 1.5f;
  float width = (float)strlen(stage->street) * 6.0f * text_scale;
  float x = ((float)stage->window->drawable_width - width) * 0.5f;
  float y = 10.0f * scale;
  texture_handle_t none = {0};
  renderer_draw_screen_image(r, none, x - 6 * scale, y - 4 * scale,
                             width + 12 * scale, 7 * text_scale + 8 * scale,
                             vec3(0.04f, 0.05f, 0.07f), 0.7f);
  renderer_draw_text(r, x + text_scale * 0.4f, y + text_scale * 0.4f,
                     text_scale, vec3(0, 0, 0), stage->street);
  renderer_draw_text(r, x, y, text_scale, vec3(1.0f, 0.97f, 0.85f),
                     stage->street);
}

/* The stars under the street name, flashing red and blue while police are
 * after the player, and a line across the screen for the news of the moment:
 * wanted, busted, lost them. */
static float distance_to_own_car(const driving_stage_t* stage);

static void draw_law(driving_stage_t* stage) {
  renderer_t* r = stage->renderer;
  float width = (float)stage->window->drawable_width;
  float scale = width / 640.0f;
  if (scale < 2) scale = 2;
  if (stage->wanted.stars > 0) {
    char line[48];
    int used = snprintf(line, sizeof(line), "WANTED ");
    for (int i = 0; i < stage->wanted.stars && used + 3 < (int)sizeof(line);
         i++)
      used += snprintf(line + used, sizeof(line) - (size_t)used, "* ");
    float text_scale = scale * 1.6f;
    float w = (float)strlen(line) * 6 * text_scale;
    bool red = fmodf(stage->rain_clock * 3.0f, 2.0f) < 1.0f;
    vec3_t colour = red ? vec3(1.0f, 0.25f, 0.2f) : vec3(0.35f, 0.5f, 1.0f);
    float x = (width - w) * 0.5f, y = 52.0f * scale;
    renderer_draw_text(r, x + text_scale * 0.4f, y + text_scale * 0.4f,
                       text_scale, vec3(0, 0, 0), line);
    renderer_draw_text(r, x, y, text_scale, colour, line);
  }
  /* What F would do now. */
  const char* hint = NULL;
  if (stage->has_figures && !stage->paused) {
    if (!stage->on_foot) {
      const rigid_body_t* car =
          physics_body(stage->physics, stage->vehicle.body);
      if (vec3_length(car->velocity) < 3.0f) hint = "F  GET OUT";
    } else if (distance_to_own_car(stage) < GET_IN_REACH) {
      hint = "F  GET IN";
    } else {
      bool near_car =
          stage->traffic && traffic_nearest_distance(
                                stage->traffic, stage->walker.position) < 4.2f;
      for (size_t i = 0; i < stage->parked.count && !near_car; i++)
        near_car =
            !stage->parked.cars[i].taken &&
            hypotf(stage->parked.cars[i].position.x - stage->walker.position.x,
                   stage->parked.cars[i].position.z -
                       stage->walker.position.z) < 4.2f;
      if (near_car) hint = "F  TAKE THE CAR";
    }
  }
  if (hint) {
    float hs = scale * 1.3f;
    float w = (float)strlen(hint) * 6 * hs;
    float hx = (width - w) * 0.5f;
    float hy = (float)stage->window->drawable_height * 0.8f;
    renderer_draw_text(r, hx + hs * 0.4f, hy + hs * 0.4f, hs, vec3(0, 0, 0),
                       hint);
    renderer_draw_text(r, hx, hy, hs, vec3(1.0f, 1.0f, 1.0f), hint);
  }
  if (stage->notice_time > 0 && stage->notice[0]) {
    float big = scale * 2.6f;
    float w = (float)strlen(stage->notice) * 6 * big;
    float x = (width - w) * 0.5f;
    float y = (float)stage->window->drawable_height * 0.34f;
    renderer_draw_text(r, x + big * 0.4f, y + big * 0.4f, big, vec3(0, 0, 0),
                       stage->notice);
    renderer_draw_text(r, x, y, big, vec3(1.0f, 0.9f, 0.3f), stage->notice);
  }
}

/* A small north-up window on the map at the top right, with the car at its
 * middle, so there is always a sense of where you are. */
static void draw_minimap(driving_stage_t* stage) {
  const city_map_t* map = &stage->scene.map;
  if (map->size == 0 || stage->map_texture.id == 0) return;
  renderer_t* r = stage->renderer;
  float scale = (float)stage->window->drawable_width / 640.0f;
  if (scale < 2) scale = 2;
  float side = 62.0f * scale;
  float x = (float)stage->window->drawable_width - side - 8.0f * scale;
  float y = 8.0f * scale;
  const rigid_body_t* body = physics_body(stage->physics, stage->vehicle.body);
  float u0, v0, u1, v1;
  city_map_window(map, body->position.x, body->position.z, 140.0f, &u0, &v0,
                  &u1, &v1);
  texture_handle_t none = {0};
  renderer_draw_screen_image(r, none, x - 2 * scale, y - 2 * scale,
                             side + 4 * scale, side + 4 * scale,
                             vec3(0.05f, 0.05f, 0.06f), 0.85f);
  renderer_draw_screen_image_uv(r, stage->map_texture, x, y, side, side, u0, v0,
                                u1, v1, vec3(1, 1, 1), 0.92f);
  /* The car: an arrow at the window's middle, or where it falls when the
   * window has been held at an edge. */
  float cx = x + ((body->position.x - map->min_x) / map->extent - u0) /
                     (u1 - u0) * side;
  float cy = y + ((body->position.z - map->min_z) / map->extent - v0) /
                     (v1 - v0) * side;
  vec3_t forward = quat_rotate(body->rotation, vec3(0, 0, -1));
  float len = sqrtf(forward.x * forward.x + forward.z * forward.z);
  float dx = len > 1e-4f ? forward.x / len : 0,
        dz = len > 1e-4f ? forward.z / len : -1;
  float arm = 4.5f * scale;
  vec3_t red = vec3(1.0f, 0.2f, 0.12f);
  renderer_draw_screen_line(r, cx - dx * arm, cy - dz * arm,
                            cx + dx * arm * 1.5f, cy + dz * arm * 1.5f,
                            2.2f * scale, vec3(1, 1, 1));
  renderer_draw_screen_line(r, cx - dx * arm, cy - dz * arm,
                            cx + dx * arm * 1.5f, cy + dz * arm * 1.5f,
                            1.2f * scale, red);
  renderer_draw_text(r, x + side * 0.5f - 2 * scale, y + 1 * scale,
                     scale * 0.9f, vec3(1, 0.9f, 0.4f), "N");
  if (stage->missions) {
    mission_route_on_minimap(r, map, stage->satnav, stage->satnav_count, x, y,
                             side, u0, v0, u1, v1, scale);
    mission_minimap_mark(&stage->mission, r, map, x, y, side, u0, v0, u1, v1,
                         scale, stage->rain_clock);
  }
}

/* The keys, along the bottom: shown for the first seconds, while paused, and
 * with the debug overlay. */
static void draw_controls_hint(driving_stage_t* stage, float x, float scale) {
  renderer_t* r = stage->renderer;
  float bottom = (float)stage->window->drawable_height - 12 * scale;
  renderer_draw_text(r, x, bottom, scale * 0.9f, vec3(0.6f, 0.6f, 0.6f),
                     stage->has_menu
                         ? "ARROWS/WASD DRIVE  SPACE HANDBRAKE  R RESET  P "
                           "PAUSE  F1 DEBUG  M MAP  N MUTE  BKSP MENU  ESC QUIT"
                         : "ARROWS/WASD DRIVE  SPACE HANDBRAKE  R RESET  P "
                           "PAUSE  F1 DEBUG  M MAP  N MUTE  F11 FULLSCREEN  "
                           "ESC QUIT");
}

/* Centred text. */
static void gauge_text(renderer_t* r, float cx, float y, float scale,
                       vec3_t colour, const char* text) {
  float width = (float)strlen(text) * 6.0f * scale;
  renderer_draw_text(r, cx - width * 0.5f + scale * 0.4f, y + scale * 0.4f,
                     scale, vec3(0, 0, 0), text);
  renderer_draw_text(r, cx - width * 0.5f, y, scale, colour, text);
}

/* The dial in the corner: a ring of segments that light up with the engine's
 * revs, green to red, the speed in the middle with the gear below it, and a
 * strip for damage. */
static void draw_gauge(driving_stage_t* stage) {
  renderer_t* r = stage->renderer;
  const vehicle_t* car = &stage->vehicle;
  float width = (float)stage->window->drawable_width;
  float height = (float)stage->window->drawable_height;
  float scale = width / 640.0f;
  if (scale < 2) scale = 2;
  float radius = 40.0f * scale;
  float cx = width - radius - 10.0f * scale;
  float cy = height - radius - 18.0f * scale;
  /* The face: a disc drawn as fat strokes from the middle. */
  vec3_t face = vec3(0.04f, 0.045f, 0.06f);
  for (int k = 0; k < 40; k++) {
    float a = 6.2831853f * (float)k / 40.0f;
    renderer_draw_screen_line(r, cx, cy, cx + cosf(a) * radius * 0.95f,
                              cy + sinf(a) * radius * 0.95f, radius * 0.2f,
                              face);
  }
  float fraction = fminf(vehicle_rpm(car) / car->params.engine_max_rpm, 1.0f);
  const int segments = 32;
  for (int i = 0; i < segments; i++) {
    float t = (float)i / (float)(segments - 1);
    /* From the lower left over the top to the lower right. */
    float a = (0.75f + 1.5f * t) * 3.1415927f;
    bool lit = t <= fraction;
    vec3_t colour = t < 0.6f    ? vec3(0.3f, 0.9f, 0.4f)
                    : t < 0.82f ? vec3(1.0f, 0.8f, 0.2f)
                                : vec3(1.0f, 0.25f, 0.2f);
    if (!lit) colour = vec3_scale(colour, 0.18f);
    float inner = radius * 0.8f, outer = radius * 0.94f;
    renderer_draw_screen_line(r, cx + cosf(a) * inner, cy + sinf(a) * inner,
                              cx + cosf(a) * outer, cy + sinf(a) * outer,
                              radius * 0.075f, colour);
  }
  char line[32];
  snprintf(line, sizeof(line), "%.0f", fabsf(vehicle_speed_kmh(car)));
  gauge_text(r, cx, cy - radius * 0.34f, scale * 2.4f, vec3(1, 1, 1), line);
  gauge_text(r, cx, cy + radius * 0.14f, scale * 0.8f, vec3(0.7f, 0.75f, 0.85f),
             "KM/H");
  int gear = vehicle_gear(car);
  if (gear == VEHICLE_GEAR_REVERSE)
    snprintf(line, sizeof(line), "R");
  else
    snprintf(line, sizeof(line), "%d", gear);
  gauge_text(r, cx, cy + radius * 0.36f, scale * 1.5f, vec3(1.0f, 0.85f, 0.4f),
             line);
  if (car->handbrake)
    gauge_text(r, cx - radius * 0.62f, cy + radius * 0.42f, scale * 1.2f,
               vec3(1.0f, 0.3f, 0.25f), "P");
  if (car->damage > 0.02f) {
    float bar = radius * 1.7f, y = cy + radius + 4.0f * scale;
    renderer_draw_screen_line(r, cx - bar * 0.5f, y, cx + bar * 0.5f, y,
                              3.0f * scale, vec3(0.12f, 0.12f, 0.14f));
    renderer_draw_screen_line(
        r, cx - bar * 0.5f, y, cx - bar * 0.5f + bar * car->damage, y,
        3.0f * scale, vec3(1.0f, 0.35f + 0.4f * (1.0f - car->damage), 0.25f));
  }
}

static void draw_hud(driving_stage_t* stage) {
  renderer_t* r = stage->renderer;
  const vehicle_t* car = &stage->vehicle;
  char line[128];
  float scale = (float)stage->window->drawable_width / 640.0f;
  if (scale < 2) scale = 2;
  float x = 16 * scale / 2, y = 16 * scale / 2;
  draw_gauge(stage);
  if (!stage->show_debug) {
    if (stage->paused)
      renderer_draw_text(r,
                         (float)stage->window->drawable_width / 2 - 30 * scale,
                         (float)stage->window->drawable_height / 2, scale * 3,
                         vec3(1, 1, 0.3f), "PAUSED");
    if (stage->paused || stage->simulated_seconds < 15.0f)
      draw_controls_hint(stage, x, scale);
    return;
  }
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
  if (car->damage > 0.02f) {
    snprintf(line, sizeof(line), "DAMAGE %3.0f%%", car->damage * 100.0f);
    renderer_draw_text(r, x + 380 * scale / 2.5f, y - 11 * scale, scale * 1.2f,
                       vec3(1.0f, 0.35f + 0.4f * (1.0f - car->damage), 0.25f),
                       line);
  }
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
  draw_controls_hint(stage, x, scale);
}

void driving_stage_set_rain(driving_stage_t* stage, float rain) {
  stage->rain_target = rain < 0 ? 0 : (rain > 1 ? 1 : rain);
}

bool driving_stage_set_vehicle(driving_stage_t* stage, int index,
                               vec3_t position, quat_t rotation) {
  vehicle_params_t params;
  if (!vehicle_params_load(&params, fleet_vehicle_file(index))) return false;
  vec3_t spawn = stage->vehicle.spawn_position;
  quat_t spawn_rotation = stage->vehicle.spawn_rotation;
  vehicle_params_t previous = stage->vehicle.params;
  vehicle_destroy(&stage->vehicle, stage->physics);
  if (!vehicle_create(&stage->vehicle, stage->physics, &params, position,
                      rotation)) {
    /* The old one goes back, so there is always a car to drive. */
    vehicle_create(&stage->vehicle, stage->physics, &previous, position,
                   rotation);
    vehicle_set_spawn(&stage->vehicle, spawn, spawn_rotation);
    return false;
  }
  vehicle_set_spawn(&stage->vehicle, spawn, spawn_rotation);
  car_visual_destroy(&stage->car_visual, stage->renderer);
  stage->car_visual = car_visual_create(stage->renderer, &params);
  stage->camera.params = chase_camera_params_for(&params);
  stage->car_camera_params = stage->camera.params;
  stage->vehicle_index = index;
  return true;
}

void driving_stage_next_vehicle(driving_stage_t* stage) {
  const rigid_body_t* body = physics_body(stage->physics, stage->vehicle.body);
  driving_stage_set_vehicle(stage, (stage->vehicle_index + 1) % FLEET_VARIANTS,
                            vec3_add(body->position, vec3(0, 0.3f, 0)),
                            body->rotation);
}

/* The camera while walking: close behind, at shoulder height. */
static chase_camera_params_t walking_camera_params(void) {
  chase_camera_params_t p = chase_camera_default_params();
  p.distance = 3.6f;
  p.height = 1.1f;
  p.look_height = 0.4f;
  p.look_ahead = 0.0f;
  p.max_look_ahead = 0.0f;
  p.fov_y = 1.0f;
  return p;
}

/* The nearest car the walker could get into: the one left behind, or any
 * other within reach. Returns 0 for none, 1 for the player's own car, 2 for
 * another. */
static float distance_to_own_car(const driving_stage_t* stage) {
  const rigid_body_t* car = physics_body(stage->physics, stage->vehicle.body);
  return hypotf(car->position.x - stage->walker.position.x,
                car->position.z - stage->walker.position.z) -
         stage->vehicle.params.chassis_half_extents.z * 0.6f;
}

void driving_stage_interact(driving_stage_t* stage) {
  if (stage->paused || stage->map_open || !stage->has_figures) return;
  const rigid_body_t* car = physics_body(stage->physics, stage->vehicle.body);
  if (!stage->on_foot) {
    if (vec3_length(car->velocity) > 3.0f) return; /* not while moving */
    /* Out on the car's left, by the driver's door. */
    vec3_t nose = quat_rotate(car->rotation, vec3(0, 0, -1));
    vec3_t left = vec3(nose.z, 0, -nose.x);
    float side = stage->vehicle.params.chassis_half_extents.x + 0.9f;
    vec3_t feet = vec3(car->position.x + left.x * side, car->position.y + 2.0f,
                       car->position.z + left.z * side);
    ray_t down = {feet, vec3(0, -1, 0)};
    ray_cast_result_t hit;
    if (physics_ray_cast(stage->physics, down, 6.0f, COLLISION_LAYER_GROUND,
                         &hit))
      feet.y = hit.hit.point.y;
    walker_init(&stage->walker, feet, atan2f(-nose.x, -nose.z));
    stage->on_foot = true;
    stage->car_camera_params = stage->camera.params;
    stage->camera.params = walking_camera_params();
    chase_camera_reset(&stage->camera,
                       transform_at(vec3_add(feet, vec3(0, 0.9f, 0)),
                                    quat_from_yaw(stage->walker.yaw)));
    return;
  }
  /* Back into the car left behind, if it is close. */
  if (distance_to_own_car(stage) < GET_IN_REACH) {
    stage->on_foot = false;
    stage->camera.params = stage->car_camera_params;
    chase_camera_reset(&stage->camera,
                       transform_at(car->position, car->rotation));
    return;
  }
  /* Or into somebody else's: a car at the kerb, or one on the road. */
  int variant = 0;
  vec3_t position;
  float yaw = 0;
  bool got =
      stage->has_fleet &&
      parked_cars_take(&stage->parked, stage->physics, stage->walker.position,
                       GET_IN_REACH + 1.0f, &variant, &position, &yaw);
  if (!got && stage->traffic)
    got = traffic_take_car(stage->traffic, stage->walker.position,
                           GET_IN_REACH + 1.0f, &variant, &position, &yaw);
  if (!got) return;
  if (!driving_stage_set_vehicle(stage, variant,
                                 vec3_add(position, vec3(0, 0.3f, 0)),
                                 quat_from_yaw(yaw)))
    return;
  stage->on_foot = false;
  chase_camera_reset(&stage->camera,
                     transform_at(position, quat_from_yaw(yaw)));
  say(stage, "CAR STOLEN");
  if (stage->missions && stage->traffic &&
      wanted_crime(&stage->wanted) == WANTED_STARTED)
    say(stage, "WANTED");
}

static bool is_a_road_surface(const char* label) {
  static const char* names[] = {"road", "junction", "pavement", "paving",
                                "parking"};
  for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
    size_t n = strlen(names[i]);
    if (strncmp(label, names[i], n) == 0 && label[n] == ' ') return true;
  }
  return false;
}

float driving_stage_wheel_angle(const driving_stage_t* stage) {
  return stage->vehicle.steering * 3.0f * (float)M_PI * 0.5f;
}

/* A hard impact jolts the view: a quick tremor that dies away. */
static render_camera_t shaken(render_camera_t view, float shake, float time) {
  if (shake < 0.01f) return view;
  float amount = shake * shake * 0.35f;
  vec3_t jolt = vec3(sinf(time * 53.0f), sinf(time * 67.0f + 1.0f),
                     sinf(time * 59.0f + 2.0f));
  jolt = vec3_scale(jolt, amount);
  view.eye = vec3_add(view.eye, jolt);
  view.target = vec3_add(view.target, vec3_scale(jolt, 0.5f));
  return view;
}

static render_camera_t driving_stage_camera_steady(
    const driving_stage_t* stage);

render_camera_t driving_stage_camera(const driving_stage_t* stage) {
  return shaken(driving_stage_camera_steady(stage), stage->shake,
                stage->rain_clock);
}

static render_camera_t driving_stage_camera_steady(
    const driving_stage_t* stage) {
  if (stage->view_mode == 0 || stage->on_foot) {
    /* The world rushes past: the view opens up a little with speed. */
    render_camera_t view = chase_camera_render(&stage->camera);
    float speed = fabsf(vehicle_forward_speed(&stage->vehicle));
    view.fov_y += fminf(speed * 0.004f, 0.14f);
    return view;
  }
  transform_t chassis =
      car_transform((driving_stage_t*)stage, (float)stage->clock.alpha);
  /* Left-hand drive: the driver sits left of centre, eyes just above the
   * sill. The bonnet view sits forward of the windscreen, on the centre line.
   */
  vec3_t local_eye = stage->view_mode == 2 ? vec3(-0.36f, 0.42f, 0.25f)
                                           : vec3(0.0f, 0.78f, -0.9f);
  vec3_t eye = transform_point(chassis, local_eye);
  vec3_t forward = quat_rotate(chassis.rotation, vec3(0, 0, -1));
  render_camera_t view = {eye,
                          vec3_add(eye, vec3_scale(forward, 20.0f)),
                          quat_rotate(chassis.rotation, vec3(0, 1, 0)),
                          stage->view_mode == 2 ? 1.25f : 1.15f,
                          0.1f,
                          1500.0f};
  return view;
}

void driving_stage_render(driving_stage_t* stage) {
  renderer_t* r = stage->renderer;
  transform_t chassis = car_transform(stage, (float)stage->clock.alpha);
  float night = daylight_lamp_glow(stage->hour);
  render_environment_t environment = daylight_environment(stage->hour);
  environment.fog_start = stage->scene.environment.fog_start;
  environment.fog_end = stage->scene.environment.fog_end;
  environment.shadow_radius = stage->scene.environment.shadow_radius;
  daylight_apply_weather(&environment, stage->rain);
  environment.cloud_time = stage->rain_clock;
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
    if (!entity->used) continue;
    if (strncmp(entity->label, "lamp_light", 10) == 0) {
      entity->material.emissive = 2.4f * night;
    } else if (strncmp(entity->label, "signal_", 7) == 0) {
      int group = entity->label[8] == '2' ? 1 : 0;
      float phase = fmodf(stage->signal_clock + 24.0f * (float)group, 48.0f);
      bool red = phase < 24.0f,
           amber = (phase >= 21.0f && phase < 24.0f) || phase >= 45.0f,
           green = phase >= 24.0f && phase < 45.0f;
      bool lit = entity->label[7] == 'r'
                     ? red
                     : (entity->label[7] == 'a' ? amber : green);
      entity->material.emissive = lit ? 2.6f : 0.12f;
    } else if (is_a_road_surface(entity->label)) {
      /* Wet tarmac: smoother, a little shine, reflecting the sky. */
      entity->material.roughness = 0.95f - 0.72f * stage->rain;
      entity->material.metallic = 0.4f * stage->rain;
    } else if (strncmp(entity->label, "facade", 6) == 0 ||
               strncmp(entity->label, "shopfront", 9) == 0) {
      entity->material.window_glow = night;
    }
  }
  renderer_begin_frame(r, driving_stage_camera(stage), environment);
  stage->last_drawn = world_render(&stage->world, r);
  car_visual_draw(&stage->car_visual, r, &stage->vehicle, chassis, night,
                  stage->rain_clock);
  car_visual_draw_smoke(&stage->car_visual, r, chassis, stage->vehicle.damage,
                        stage->rain_clock);
  if (stage->view_mode == 2)
    car_visual_draw_cockpit(&stage->car_visual, r, chassis,
                            driving_stage_wheel_angle(stage));
  if (stage->rain > 0.04f) {
    /* Drops: short streaks falling in a box about the eye, each at its own
     * phase, leaning a little with the wind. */
    int drops = (int)(900.0f * stage->rain);
    vec3_t eye = stage->camera.position;
    for (int i = 0; i < drops; i++) {
      unsigned h = (unsigned)i * 2654435761u;
      float x = ((h >> 3) % 1000) / 1000.0f * 36.0f - 18.0f;
      float z = ((h >> 13) % 1000) / 1000.0f * 36.0f - 18.0f;
      float phase = (float)((h >> 23) % 1000) / 1000.0f;
      float y = 14.0f - fmodf(phase * 14.0f + stage->rain_clock * 13.0f, 14.0f);
      vec3_t top = vec3(eye.x + x, eye.y - 4.0f + y, eye.z + z);
      renderer_draw_line_tested(r, top,
                                vec3_add(top, vec3(0.12f, -0.7f, 0.05f)),
                                vec3(0.62f, 0.68f, 0.78f));
    }
  }
  skid_marks_draw(&stage->skid_marks, stage->skid_strip, r,
                  stage->camera.position, 90.0f);
  if (stage->has_fleet) {
    vec3_t eye = stage->camera.position;
    parked_cars_draw(&stage->parked, &stage->fleet, r, eye, 300.0f, night);
    if (stage->people)
      pedestrians_draw(stage->people, &stage->people_meshes, r, eye, 110.0f);
    if (stage->traffic)
      traffic_draw(stage->traffic, &stage->fleet, r, eye, TRAFFIC_VISIBLE_RANGE,
                   night);
  }
  if (stage->on_foot) {
    /* The person who got out, in the clothes of an adult from the crowd. */
    pedestrian_draw_figure(&stage->people_meshes, r, stage->walker.position,
                           stage->walker.yaw, stage->walker.speed,
                           stage->walker.phase, stage->walker_outfit);
  }
  if (stage->missions)
    mission_marker_draw(&stage->mission, &stage->mission_markers, r,
                        stage->mission_ground, stage->rain_clock);
  if (stage->show_debug) draw_debug(stage, chassis);
  if (stage->map_open) {
    draw_map(stage);
  } else {
    draw_hud(stage);
    if (stage->missions) {
      const rigid_body_t* me =
          physics_body(stage->physics, stage->vehicle.body);
      vec3_t heading = quat_rotate(me->rotation, vec3(0, 0, -1));
      float length = hypotf(heading.x, heading.z);
      /* The arrow follows the satnav: the first point of the way that is
       * a little ahead, else straight at the destination. */
      float aim_x = stage->mission.x, aim_z = stage->mission.z;
      for (size_t i = 0; i < stage->satnav_count; i++) {
        float ax = stage->satnav[2 * i], az = stage->satnav[2 * i + 1];
        if (hypotf(ax - me->position.x, az - me->position.z) > 35.0f) {
          aim_x = ax, aim_z = az;
          break;
        }
      }
      mission_hud_draw(&stage->mission, r, (float)stage->window->drawable_width,
                       (float)stage->window->drawable_height, me->position.x,
                       me->position.z, length > 1e-4f ? heading.x / length : 0,
                       length > 1e-4f ? heading.z / length : -1, aim_x, aim_z);
    }
    draw_minimap(stage);
    draw_street_name(stage);
    draw_law(stage);
  }
  renderer_end_frame(r);
}

static void handle_events(driving_stage_t* stage, window_events_t events) {
  if (events.quit) stage->quit = true;
  if (events.resized)
    renderer_resize(stage->renderer, stage->window->drawable_width,
                    stage->window->drawable_height);
  if (events.reset_vehicle) back_to_spawn(stage);
  if (events.back_to_menu && stage->has_menu) {
    stage->back_to_menu = true;
    stage->quit = true;
  }
  if (events.interact) driving_stage_interact(stage);
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
  if (events.next_vehicle) driving_stage_next_vehicle(stage);
  if (events.toggle_weather)
    driving_stage_set_rain(stage, stage->rain_target > 0.5f ? 0.0f : 1.0f);
  if (events.toggle_camera) {
    stage->view_mode = (stage->view_mode + 1) % 3;
    if (stage->view_mode == 0)
      chase_camera_reset(&stage->camera, car_transform(stage, 1.0f));
  }
}

void driving_stage_run(driving_stage_t* stage) {
  double previous = platform_seconds();
  while (!stage->quit) {
    double now = platform_seconds();
    double frame_seconds = now - previous;
    previous = now;
    handle_events(stage, window_poll(stage->window));
    driving_input_t input = window_driving_input(stage->window);
    stage->horn = window_horn_held(stage->window);
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
