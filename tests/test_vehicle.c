#include <assert.h>
#include <math.h>

#include "physics_world.h"
#include "test_groups.h"
#include "test_support.h"
#include "vehicle.h"

static const float step = 1.0f / 120.0f;

typedef struct {
  physics_world_t* world;
  vehicle_t car;
} track_t;

static track_t flat_track(float friction) {
  track_t track;
  track.world = physics_world_create();
  vec3_t vertices[4] = {vec3(-500, 0, 500), vec3(500, 0, 500),
                        vec3(500, 0, -500), vec3(-500, 0, -500)};
  uint32_t indices[6] = {0, 1, 2, 0, 2, 3};
  physics_add_static_mesh(track.world, vertices, 4, indices, 6,
                          COLLISION_LAYER_GROUND, friction, 0);
  vehicle_params_t params;
  bool loaded = vehicle_params_load(&params, "assets/vehicles/hatchback.cfg");
  assert(loaded);
  assert(vehicle_create(&track.car, track.world, &params, vec3(0, 0.9f, 0),
                        quat_identity()));
  return track;
}

static void drive(track_t* track, driving_input_t input, float seconds) {
  int steps = (int)(seconds / step);
  for (int i = 0; i < steps; i++) {
    vehicle_update(&track->car, track->world, input, step);
    physics_world_step(track->world, step);
  }
}

static driving_input_t idle(void) {
  driving_input_t input = {0, 0, 0, false};
  return input;
}

static void destroy(track_t* track) { physics_world_destroy(track->world); }

static void parameters_load_from_the_tuning_file(void) {
  vehicle_params_t params;
  assert(vehicle_params_load(&params, "assets/vehicles/hatchback.cfg"));
  assert(near(params.mass, 1150));
  assert(params.gear_count == 5);
  assert(near(params.gear_ratios[0], 2.8f));
  assert(params.torque_point_count == 7);
  assert(near_to(vehicle_engine_torque(&params, 4000), 170, 1.0f));
  assert(near_to(vehicle_engine_torque(&params, 7000), 0, 1e-6));
  assert(params.drive == VEHICLE_DRIVE_FRONT);
  assert(!vehicle_params_load(&params, "assets/vehicles/missing.cfg"));
}

static void the_car_settles_on_its_wheels(void) {
  track_t track = flat_track(1.0f);
  drive(&track, idle(), 3.0f);
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
    const wheel_state_t* wheel = &track.car.wheels[i];
    assert(wheel->grounded);
    assert(wheel->compression > 0.03f && wheel->compression < 0.2f);
  }
  const rigid_body_t* body = physics_body(track.world, track.car.body);
  assert(quat_up(body->rotation).y > 0.999f);
  assert(fabsf(vehicle_forward_speed(&track.car)) < 0.05f);
  assert(vehicle_rpm(&track.car) >= 900);
  assert(vehicle_gear(&track.car) == 1);
  destroy(&track);
}

static void full_throttle_reaches_fifty_in_a_plausible_time(void) {
  track_t track = flat_track(1.0f);
  drive(&track, idle(), 2.0f);
  driving_input_t input = idle();
  input.throttle = 1;
  float seconds = 0;
  while (vehicle_forward_speed(&track.car) < 50.0f / 3.6f && seconds < 20.0f) {
    vehicle_update(&track.car, track.world, input, step);
    physics_world_step(track.world, step);
    seconds += step;
  }
  assert(seconds > 2.0f && seconds < 10.0f);
  drive(&track, input, 4.0f); /* well past 50 km/h the gearbox has shifted up */
  assert(vehicle_gear(&track.car) >= 2);
  const rigid_body_t* body = physics_body(track.world, track.car.body);
  /* Straight line: forward is -Z. */
  assert(body->position.z < -10.0f);
  assert(fabsf(body->position.x) < 1.0f);
  destroy(&track);
}

static void braking_from_speed_stops_within_a_plausible_distance(void) {
  track_t track = flat_track(1.0f);
  drive(&track, idle(), 2.0f);
  rigid_body_t* body = physics_body(track.world, track.car.body);
  body->velocity = vec3(0, 0, -20);
  float start = body->position.z;
  driving_input_t input = idle();
  input.brake = 1;
  float seconds = 0;
  do {
    vehicle_update(&track.car, track.world, input, step);
    physics_world_step(track.world, step);
    seconds += step;
  } while (vehicle_forward_speed(&track.car) > 0.2f && seconds < 10.0f);
  float distance = start - body->position.z;
  assert(seconds < 5.0f);
  assert(distance > 15.0f && distance < 45.0f);
  destroy(&track);
}

static void steering_left_turns_the_heading_left(void) {
  track_t track = flat_track(1.0f);
  drive(&track, idle(), 2.0f);
  rigid_body_t* body = physics_body(track.world, track.car.body);
  body->velocity = vec3(0, 0, -10);
  driving_input_t input = idle();
  input.throttle = 0.3f;
  input.steering = -1;
  drive(&track, input, 2.0f);
  assert(quat_yaw(body->rotation) > 0.3f);
  assert(body->position.x < -0.5f);
  assert(quat_up(body->rotation).y > 0.99f);
  destroy(&track);
}

static void holding_the_brake_at_rest_reverses(void) {
  track_t track = flat_track(1.0f);
  drive(&track, idle(), 2.0f);
  driving_input_t input = idle();
  input.brake = 1;
  drive(&track, input, 3.0f);
  assert(vehicle_gear(&track.car) == VEHICLE_GEAR_REVERSE);
  assert(vehicle_forward_speed(&track.car) < -1.0f);
  input = idle();
  input.throttle = 1;
  drive(&track, input, 3.0f);
  assert(vehicle_forward_speed(&track.car) > 1.0f);
  assert(vehicle_gear(&track.car) >= 1);
  destroy(&track);
}

static void a_slippery_surface_lengthens_the_stop(void) {
  track_t dry = flat_track(1.0f);
  track_t wet = flat_track(0.4f);
  drive(&dry, idle(), 2.0f);
  drive(&wet, idle(), 2.0f);
  physics_body(dry.world, dry.car.body)->velocity = vec3(0, 0, -15);
  physics_body(wet.world, wet.car.body)->velocity = vec3(0, 0, -15);
  driving_input_t input = idle();
  input.brake = 1;
  drive(&dry, input, 2.5f);
  drive(&wet, input, 2.5f);
  assert(vehicle_forward_speed(&dry.car) < 0.5f);
  assert(vehicle_forward_speed(&wet.car) >
         vehicle_forward_speed(&dry.car) + 2.0f);
  destroy(&dry);
  destroy(&wet);
}

static void reset_returns_the_car_to_its_spawn(void) {
  track_t track = flat_track(1.0f);
  driving_input_t input = idle();
  input.throttle = 1;
  drive(&track, input, 3.0f);
  vehicle_reset(&track.car, track.world);
  const rigid_body_t* body = physics_body(track.world, track.car.body);
  assert(near(body->position.x, 0) && near(body->position.z, 0));
  assert(near(vec3_length(body->velocity), 0));
  assert(vehicle_gear(&track.car) == 1);
  destroy(&track);
}

static void wheels_follow_the_suspension_and_spin_with_speed(void) {
  track_t track = flat_track(1.0f);
  drive(&track, idle(), 2.0f);
  const rigid_body_t* body = physics_body(track.world, track.car.body);
  transform_t chassis = transform_at(body->position, body->rotation);
  transform_t front_left = vehicle_wheel_transform(&track.car, chassis, 0);
  assert(near_to(front_left.position.y, track.car.params.wheel_radius, 0.02f));
  assert(front_left.position.z < 0 && front_left.position.x < 0);
  float before = track.car.wheels[0].rotation;
  driving_input_t input = idle();
  input.throttle = 1;
  drive(&track, input, 1.0f);
  assert(track.car.wheels[0].rotation != before);
  destroy(&track);
}

static void the_handbrake_locks_the_rear_and_marks_a_skid(void) {
  track_t track = flat_track(1.0f);
  drive(&track, idle(), 2.0f);
  physics_body(track.world, track.car.body)->velocity = vec3(0, 0, -15);
  driving_input_t input = idle();
  input.handbrake = true;
  input.steering = -1;
  bool skidded = false;
  for (int i = 0; i < 120; i++) {
    vehicle_update(&track.car, track.world, input, step);
    physics_world_step(track.world, step);
    if (vehicle_skid_intensity(&track.car) > 0.1f) skidded = true;
  }
  assert(skidded);
  destroy(&track);
}

/* How far the body has rolled about its own length, in radians. */
static float roll_angle(const track_t* track) {
  const rigid_body_t* body = physics_body(track->world, track->car.body);
  return asinf(quat_right(body->rotation).y); /* the right side's height */
}

static void a_hard_corner_leans_the_body_a_few_degrees_not_a_dozen(void) {
  track_t track = flat_track(1.0f);
  driving_input_t throttle = {1, 0, 0, false};
  drive(&track, throttle, 6.0f); /* up to speed */
  driving_input_t hold = {0.25f, 0, 0, false};
  float worst = 0;
  driving_input_t turn = {0.25f, 0, 1.0f, false};
  int steps = (int)(4.0f / step);
  for (int i = 0; i < steps; i++) {
    vehicle_update(&track.car, track.world, turn, step);
    physics_world_step(track.world, step);
    worst = fmaxf(worst, fabsf(roll_angle(&track)));
  }
  (void)hold;
  /* A hatchback pulling most of a g leans three to four degrees. */
  assert(worst < 0.075f);
  assert(worst > 0.01f); /* but it does lean */
  destroy(&track);
}

static void the_car_does_not_tip_over_in_a_sudden_swerve(void) {
  track_t track = flat_track(1.0f);
  driving_input_t throttle = {1, 0, 0, false};
  drive(&track, throttle, 7.0f);
  float worst = 0;
  for (int swerve = 0; swerve < 6; swerve++) {
    driving_input_t turn = {0.5f, 0, swerve % 2 ? -1.0f : 1.0f, false};
    for (int i = 0; i < (int)(0.5f / step); i++) {
      vehicle_update(&track.car, track.world, turn, step);
      physics_world_step(track.world, step);
      worst = fmaxf(worst, fabsf(roll_angle(&track)));
    }
  }
  assert(worst < 0.12f); /* under about 7 degrees */
  assert(quat_up(physics_body(track.world, track.car.body)->rotation).y >
         0.95f);
  destroy(&track);
}

static float pitch_angle(const track_t* track) {
  const rigid_body_t* body = physics_body(track->world, track->car.body);
  return asinf(quat_forward(body->rotation).y); /* nose up is positive */
}

static void hard_braking_dips_the_nose_a_couple_of_degrees(void) {
  track_t track = flat_track(1.0f);
  driving_input_t throttle = {1, 0, 0, false};
  drive(&track, throttle, 9.0f);
  driving_input_t brake = {0, 1, 0, false};
  float dip = 0;
  for (int i = 0; i < (int)(2.0f / step); i++) {
    vehicle_update(&track.car, track.world, brake, step);
    physics_world_step(track.world, step);
    dip = fmaxf(dip, -pitch_angle(&track));
  }
  assert(dip < 0.06f); /* under about 3.5 degrees */
  assert(dip > 0.005f);
  destroy(&track);
}

static void every_car_in_the_fleet_accelerates_corners_and_stops_sensibly(
    void) {
  const char* files[4] = {
      "assets/vehicles/hatchback.cfg", "assets/vehicles/sedan.cfg",
      "assets/vehicles/van.cfg", "assets/vehicles/mini.cfg"};
  for (int i = 0; i < 4; i++) {
    physics_world_t* world = physics_world_create();
    vec3_t v[4] = {vec3(-800, 0, 800), vec3(800, 0, 800), vec3(800, 0, -800),
                   vec3(-800, 0, -800)};
    uint32_t idx[6] = {0, 1, 2, 0, 2, 3};
    physics_add_static_mesh(world, v, 4, idx, 6, COLLISION_LAYER_GROUND, 1.0f,
                            0);
    vehicle_params_t params;
    assert(vehicle_params_load(&params, files[i]));
    vehicle_t car;
    assert(vehicle_create(&car, world, &params, vec3(0, 0.9f, 0),
                          quat_identity()));
    track_t track = {world, car};
    driving_input_t throttle = {1, 0, 0, false};
    drive(&track, throttle, 10.0f);
    float speed = vehicle_speed_kmh(&track.car);
    assert(speed > 55.0f && speed < 200.0f);
    /* A hard corner at pace leans it a few degrees, never a dozen. */
    driving_input_t turn = {0.3f, 0, 1.0f, false};
    float worst = 0;
    for (int k = 0; k < (int)(3.0f / step); k++) {
      vehicle_update(&track.car, track.world, turn, step);
      physics_world_step(track.world, step);
      worst = fmaxf(worst, fabsf(roll_angle(&track)));
    }
    assert(worst < 0.12f);
    /* And it stops. */
    driving_input_t brake = {0, 1, 0, false};
    /* Holding the brake at a standstill reverses, so look for the stop. */
    float slowest = 1e9f;
    for (int k = 0; k < (int)(8.0f / step); k++) {
      vehicle_update(&track.car, track.world, brake, step);
      physics_world_step(track.world, step);
      slowest = fminf(slowest, fabsf(vehicle_forward_speed(&track.car)));
    }
    assert(slowest < 0.6f);
    vehicle_destroy(&track.car, world);
    physics_world_destroy(world);
  }
}

void run_vehicle_tests(void) {
  RUN(every_car_in_the_fleet_accelerates_corners_and_stops_sensibly);
  RUN(hard_braking_dips_the_nose_a_couple_of_degrees);
  RUN(a_hard_corner_leans_the_body_a_few_degrees_not_a_dozen);
  RUN(the_car_does_not_tip_over_in_a_sudden_swerve);
  RUN(parameters_load_from_the_tuning_file);
  RUN(the_car_settles_on_its_wheels);
  RUN(full_throttle_reaches_fifty_in_a_plausible_time);
  RUN(braking_from_speed_stops_within_a_plausible_distance);
  RUN(steering_left_turns_the_heading_left);
  RUN(holding_the_brake_at_rest_reverses);
  RUN(a_slippery_surface_lengthens_the_stop);
  RUN(reset_returns_the_car_to_its_spawn);
  RUN(wheels_follow_the_suspension_and_spin_with_speed);
  RUN(the_handbrake_locks_the_rear_and_marks_a_skid);
}
