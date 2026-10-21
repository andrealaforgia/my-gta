#include "vehicle.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "interpolation.h"
#include "log.h"

#define GRAVITY 9.81f
#define RPM_PER_RAD_PER_SECOND (60.0f / (2.0f * (float)M_PI))
#define REVERSE_ENGAGE_SPEED 0.4f
#define SKID_SLIP_ANGLE 0.3f

/* ---- parameters ---- */

vehicle_params_t vehicle_params_default(void) {
  vehicle_params_t p;
  memset(&p, 0, sizeof(p));
  snprintf(p.name, sizeof(p.name), "default");
  snprintf(p.body, sizeof(p.body), "car");
  p.mass = 1200;
  p.chassis_half_extents = vec3(0.85f, 0.28f, 1.95f);
  p.track_width = 1.5f;
  p.wheelbase = 2.55f;
  p.wheel_attach_height = -0.12f;
  p.wheel_radius = 0.31f;
  p.wheel_width = 0.2f;
  p.suspension_rest_length = 0.28f;
  p.spring_stiffness = 32000;
  p.damper_compression = 2600;
  p.damper_rebound = 3400;
  p.tyre_friction = 1.0f;
  p.tyre_lateral_stiffness = 9.0f;
  p.rolling_resistance = 0.015f;
  p.roll_centre_lift = 0.0f;
  p.drag_coefficient = 0.4f;
  p.max_steer_angle = 0.55f;
  p.steering_rate = 7.0f;
  p.steering_half_lock_speed = 24.0f;
  p.brake_force = 11000;
  p.handbrake_force = 5000;
  p.handbrake_grip_factor = 0.55f;
  p.engine_idle_rpm = 900;
  p.engine_max_rpm = 6500;
  p.engine_braking = 0.12f;
  torque_point_t curve[] = {{900, 95}, {2500, 150}, {4500, 172}, {6500, 125}};
  p.torque_point_count = 4;
  memcpy(p.torque_curve, curve, sizeof(curve));
  float gears[] = {3.5f, 2.05f, 1.38f, 1.03f, 0.82f};
  p.gear_count = 5;
  memcpy(p.gear_ratios, gears, sizeof(gears));
  p.reverse_ratio = 3.2f;
  p.final_drive = 4.1f;
  p.drivetrain_efficiency = 0.9f;
  p.shift_up_rpm = 5800;
  p.shift_down_rpm = 2400;
  p.shift_time = 0.25f;
  p.drive = VEHICLE_DRIVE_FRONT;
  return p;
}

static int parse_torque_curve(const char* text, torque_point_t* points,
                              int capacity) {
  int count = 0;
  const char* cursor = text;
  while (*cursor && count < capacity) {
    char* end;
    float rpm = strtof(cursor, &end);
    if (end == cursor || *end != ':') break;
    cursor = end + 1;
    float torque = strtof(cursor, &end);
    if (end == cursor) break;
    points[count].rpm = rpm;
    points[count].torque = torque;
    count++;
    cursor = end;
    while (*cursor == ' ') cursor++;
  }
  return count;
}

static int parse_floats(const char* text, float* values, int capacity) {
  int count = 0;
  const char* cursor = text;
  while (*cursor && count < capacity) {
    char* end;
    float value = strtof(cursor, &end);
    if (end == cursor) break;
    values[count++] = value;
    cursor = end;
  }
  return count;
}

bool vehicle_params_load(vehicle_params_t* params, const char* path) {
  *params = vehicle_params_default();
  config_t config;
  if (!config_load(&config, path)) {
    LOG_ERROR("vehicle", "cannot read %s", path);
    config_free(&config);
    return false;
  }
  vehicle_params_t* p = params;
  snprintf(p->name, sizeof(p->name), "%s",
           config_get_string(&config, "name", p->name));
  snprintf(p->body, sizeof(p->body), "%s",
           config_get_string(&config, "body", p->body));
  p->mass = config_get_float(&config, "mass", p->mass);
  p->chassis_half_extents.x = config_get_float(&config, "chassis_half_width",
                                               p->chassis_half_extents.x);
  p->chassis_half_extents.y = config_get_float(&config, "chassis_half_height",
                                               p->chassis_half_extents.y);
  p->chassis_half_extents.z = config_get_float(&config, "chassis_half_length",
                                               p->chassis_half_extents.z);
  p->track_width = config_get_float(&config, "track_width", p->track_width);
  p->wheelbase = config_get_float(&config, "wheelbase", p->wheelbase);
  p->wheel_attach_height =
      config_get_float(&config, "wheel_attach_height", p->wheel_attach_height);
  p->wheel_radius = config_get_float(&config, "wheel_radius", p->wheel_radius);
  p->wheel_width = config_get_float(&config, "wheel_width", p->wheel_width);
  p->suspension_rest_length = config_get_float(
      &config, "suspension_rest_length", p->suspension_rest_length);
  p->spring_stiffness =
      config_get_float(&config, "spring_stiffness", p->spring_stiffness);
  p->damper_compression =
      config_get_float(&config, "damper_compression", p->damper_compression);
  p->damper_rebound =
      config_get_float(&config, "damper_rebound", p->damper_rebound);
  p->tyre_friction =
      config_get_float(&config, "tyre_friction", p->tyre_friction);
  p->tyre_lateral_stiffness = config_get_float(
      &config, "tyre_lateral_stiffness", p->tyre_lateral_stiffness);
  p->rolling_resistance =
      config_get_float(&config, "rolling_resistance", p->rolling_resistance);
  p->roll_centre_lift =
      config_get_float(&config, "roll_centre_lift", p->roll_centre_lift);
  p->drag_coefficient =
      config_get_float(&config, "drag_coefficient", p->drag_coefficient);
  p->max_steer_angle =
      config_get_float(&config, "max_steer_angle", p->max_steer_angle);
  p->steering_rate =
      config_get_float(&config, "steering_rate", p->steering_rate);
  p->steering_half_lock_speed = config_get_float(
      &config, "steering_half_lock_speed", p->steering_half_lock_speed);
  p->brake_force = config_get_float(&config, "brake_force", p->brake_force);
  p->handbrake_force =
      config_get_float(&config, "handbrake_force", p->handbrake_force);
  p->handbrake_grip_factor = config_get_float(&config, "handbrake_grip_factor",
                                              p->handbrake_grip_factor);
  p->engine_idle_rpm =
      config_get_float(&config, "engine_idle_rpm", p->engine_idle_rpm);
  p->engine_max_rpm =
      config_get_float(&config, "engine_max_rpm", p->engine_max_rpm);
  p->engine_braking =
      config_get_float(&config, "engine_braking", p->engine_braking);
  if (config_has(&config, "torque_curve")) {
    p->torque_point_count =
        parse_torque_curve(config_get_string(&config, "torque_curve", ""),
                           p->torque_curve, VEHICLE_MAX_TORQUE_POINTS);
  }
  if (config_has(&config, "gear_ratios")) {
    p->gear_count = parse_floats(config_get_string(&config, "gear_ratios", ""),
                                 p->gear_ratios, VEHICLE_MAX_GEARS);
  }
  p->reverse_ratio =
      config_get_float(&config, "reverse_ratio", p->reverse_ratio);
  p->final_drive = config_get_float(&config, "final_drive", p->final_drive);
  p->drivetrain_efficiency = config_get_float(&config, "drivetrain_efficiency",
                                              p->drivetrain_efficiency);
  p->shift_up_rpm = config_get_float(&config, "shift_up_rpm", p->shift_up_rpm);
  p->shift_down_rpm =
      config_get_float(&config, "shift_down_rpm", p->shift_down_rpm);
  p->shift_time = config_get_float(&config, "shift_time", p->shift_time);
  const char* drive = config_get_string(&config, "drive", "front");
  p->drive = strcmp(drive, "rear") == 0  ? VEHICLE_DRIVE_REAR
             : strcmp(drive, "all") == 0 ? VEHICLE_DRIVE_ALL
                                         : VEHICLE_DRIVE_FRONT;
  if (config_defaulted_count(&config) > 0)
    LOG_WARN("vehicle", "%s: %zu parameters missing, defaults used", path,
             config_defaulted_count(&config));
  if (config_malformed_count(&config) > 0)
    LOG_WARN("vehicle", "%s: %zu malformed lines ignored", path,
             config_malformed_count(&config));
  bool valid = p->torque_point_count >= 2 && p->gear_count >= 1 &&
               p->mass > 0 && p->wheel_radius > 0;
  if (!valid)
    LOG_ERROR("vehicle",
              "%s: torque curve, gears, mass and wheel radius are required",
              path);
  config_free(&config);
  return valid;
}

float vehicle_engine_torque(const vehicle_params_t* params, float rpm) {
  if (params->torque_point_count == 0) return 0;
  if (rpm > params->engine_max_rpm) return 0;
  if (rpm <= params->torque_curve[0].rpm) return params->torque_curve[0].torque;
  for (int i = 1; i < params->torque_point_count; i++) {
    const torque_point_t* a = &params->torque_curve[i - 1];
    const torque_point_t* b = &params->torque_curve[i];
    if (rpm <= b->rpm) {
      float t = (rpm - a->rpm) / (b->rpm - a->rpm);
      return lerpf(a->torque, b->torque, t);
    }
  }
  return params->torque_curve[params->torque_point_count - 1].torque;
}

/* ---- lifetime ---- */

static void place_wheels(vehicle_t* vehicle) {
  const vehicle_params_t* p = &vehicle->params;
  float x = p->track_width * 0.5f, y = p->wheel_attach_height,
        z = p->wheelbase * 0.5f;
  /* Order: front left, front right, rear left, rear right. Front is -Z. */
  vec3_t attach[VEHICLE_WHEEL_COUNT] = {vec3(-x, y, -z), vec3(x, y, -z),
                                        vec3(-x, y, z), vec3(x, y, z)};
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
    wheel_state_t* wheel = &vehicle->wheels[i];
    memset(wheel, 0, sizeof(*wheel));
    wheel->attach_local = attach[i];
    bool front = i < 2;
    wheel->steerable = front;
    wheel->handbraked = !front;
    wheel->driven = p->drive == VEHICLE_DRIVE_ALL ||
                    (front ? p->drive == VEHICLE_DRIVE_FRONT
                           : p->drive == VEHICLE_DRIVE_REAR);
    wheel->contact_normal = vec3(0, 1, 0);
  }
}

bool vehicle_create(vehicle_t* vehicle, physics_world_t* world,
                    const vehicle_params_t* params, vec3_t position,
                    quat_t rotation) {
  memset(vehicle, 0, sizeof(*vehicle));
  vehicle->params = *params;
  vehicle->body = physics_add_body(world, position, rotation, params->mass,
                                   params->chassis_half_extents);
  if (vehicle->body.id == 0) return false;
  rigid_body_t* body = physics_body(world, vehicle->body);
  body->restitution = 0.1f;
  body->angular_damping = 0.5f;
  vehicle->spawn_position = position;
  vehicle->spawn_rotation = rotation;
  vehicle->gear = 1;
  vehicle->rpm = params->engine_idle_rpm;
  vehicle->steering_filter = steering_filter_create(
      params->steering_rate, params->steering_half_lock_speed);
  place_wheels(vehicle);
  return true;
}

void vehicle_destroy(vehicle_t* vehicle, physics_world_t* world) {
  physics_remove_body(world, vehicle->body);
  vehicle->body.id = 0;
}

void vehicle_set_spawn(vehicle_t* vehicle, vec3_t position, quat_t rotation) {
  vehicle->spawn_position = position;
  vehicle->spawn_rotation = rotation;
}

void vehicle_reset(vehicle_t* vehicle, physics_world_t* world) {
  physics_body_teleport(world, vehicle->body, vehicle->spawn_position,
                        vehicle->spawn_rotation);
  vehicle->gear = 1;
  vehicle->rpm = vehicle->params.engine_idle_rpm;
  vehicle->shift_timer = 0;
  vehicle->steering_filter.value = 0;
  vehicle->forward_speed = 0;
  vehicle->skid_intensity = 0;
  vehicle->damage = 0;
  place_wheels(vehicle);
}

/* ---- per-step simulation ---- */

static int driven_wheel_count(const vehicle_t* vehicle) {
  int count = 0;
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++)
    count += vehicle->wheels[i].driven ? 1 : 0;
  return count > 0 ? count : 1;
}

static float current_ratio(const vehicle_t* vehicle) {
  const vehicle_params_t* p = &vehicle->params;
  float gear = vehicle->gear == VEHICLE_GEAR_REVERSE
                   ? -p->reverse_ratio
                   : p->gear_ratios[vehicle->gear - 1];
  return gear * p->final_drive;
}

static void update_gearbox(vehicle_t* vehicle, float throttle, float brake,
                           float dt) {
  const vehicle_params_t* p = &vehicle->params;
  float speed = vehicle->forward_speed;
  if (vehicle->shift_timer > 0) vehicle->shift_timer -= dt;
  if (vehicle->gear == VEHICLE_GEAR_REVERSE) {
    if (throttle > 0.1f && speed > -REVERSE_ENGAGE_SPEED) vehicle->gear = 1;
    return;
  }
  if (brake > 0.1f && throttle < 0.1f && speed < REVERSE_ENGAGE_SPEED) {
    vehicle->gear = VEHICLE_GEAR_REVERSE;
    return;
  }
  if (vehicle->shift_timer > 0) return;
  if (vehicle->rpm > p->shift_up_rpm && vehicle->gear < p->gear_count) {
    vehicle->gear++;
    vehicle->shift_timer = p->shift_time;
  } else if (vehicle->rpm < p->shift_down_rpm && vehicle->gear > 1) {
    vehicle->gear--;
    vehicle->shift_timer = p->shift_time;
  }
}

static void update_engine(vehicle_t* vehicle) {
  const vehicle_params_t* p = &vehicle->params;
  /* Engine speed follows the driven wheels; below idle the clutch slips. */
  float wheel_rate = 0;
  int grounded_driven = 0;
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
    const wheel_state_t* wheel = &vehicle->wheels[i];
    if (!wheel->driven) continue;
    wheel_rate += wheel->spin;
    grounded_driven++;
  }
  if (grounded_driven > 0) wheel_rate /= (float)grounded_driven;
  float rpm =
      fabsf(wheel_rate * current_ratio(vehicle)) * RPM_PER_RAD_PER_SECOND;
  if (rpm < p->engine_idle_rpm) rpm = p->engine_idle_rpm;
  if (rpm > p->engine_max_rpm) rpm = p->engine_max_rpm;
  vehicle->rpm = rpm;
}

static float drive_force_per_wheel(const vehicle_t* vehicle, float throttle) {
  const vehicle_params_t* p = &vehicle->params;
  if (vehicle->shift_timer > 0) return 0;
  float ratio = current_ratio(vehicle);
  float torque = vehicle_engine_torque(p, vehicle->rpm) * throttle *
                 vehicle_power_factor(vehicle);
  if (throttle < 0.05f) {
    /* Engine braking grows with revs and only acts while the wheels drive
     * the engine, so a stopped car is not dragged backwards. */
    float peak = vehicle_engine_torque(p, 4000);
    float fraction = (vehicle->rpm - p->engine_idle_rpm) /
                     (p->engine_max_rpm - p->engine_idle_rpm);
    torque = -p->engine_braking * peak * fmaxf(fraction, 0.0f);
    if (vehicle->gear == VEHICLE_GEAR_REVERSE) torque = -torque;
    if (fabsf(vehicle->forward_speed) < 0.5f) torque = 0;
  }
  return torque * ratio * p->drivetrain_efficiency / p->wheel_radius /
         (float)driven_wheel_count(vehicle);
}

static void update_wheel(vehicle_t* vehicle, physics_world_t* world,
                         rigid_body_t* body, int index, float drive_force,
                         float brake, float dt) {
  const vehicle_params_t* p = &vehicle->params;
  wheel_state_t* wheel = &vehicle->wheels[index];
  vec3_t up = quat_up(body->rotation);
  vec3_t attach = vec3_add(body->position,
                           quat_rotate(body->rotation, wheel->attach_local));
  ray_t ray = {attach, vec3_negate(up)};
  float reach = p->suspension_rest_length + p->wheel_radius;
  ray_cast_result_t hit;
  bool grounded =
      physics_ray_cast(world, ray, reach, COLLISION_LAYER_ALL, &hit);
  float previous_compression = wheel->compression;
  if (!grounded) {
    wheel->grounded = false;
    wheel->compression =
        fmaxf(wheel->compression - 2.0f * dt, 0.0f); /* rebound in the air */
    wheel->compression_rate = 0;
    wheel->load = 0;
    wheel->grip_use = 0;
    wheel->skidding = false;
    wheel->hub_position = vec3_add_scaled(
        attach, up, -(p->suspension_rest_length - wheel->compression));
    wheel->spin *= 0.995f; /* slowly loses speed in the air */
    wheel->rotation += wheel->spin * dt;
    return;
  }
  wheel->grounded = true;
  wheel->contact_point = hit.hit.point;
  wheel->contact_normal = hit.hit.normal;
  wheel->surface_friction = hit.friction;
  wheel->compression =
      clampf(reach - hit.hit.distance, 0.0f, p->suspension_rest_length);
  wheel->compression_rate = (wheel->compression - previous_compression) / dt;
  float damping =
      wheel->compression_rate > 0 ? p->damper_compression : p->damper_rebound;
  float spring = p->spring_stiffness * wheel->compression +
                 damping * wheel->compression_rate;
  if (spring < 0) spring = 0;
  wheel->load = spring;
  wheel->hub_position = vec3_add_scaled(
      attach, up, -(p->suspension_rest_length - wheel->compression));
  physics_apply_force_at_world_point(world, vehicle->body,
                                     vec3_scale(up, spring), attach);

  /* Tyre axes on the contact plane. Steering is a yaw of the local forward. */
  float steer = wheel->steer_angle;
  vec3_t local_forward = vec3(-sinf(steer), 0, -cosf(steer));
  vec3_t forward = quat_rotate(body->rotation, local_forward);
  vec3_t n = wheel->contact_normal;
  forward =
      vec3_normalize(vec3_sub(forward, vec3_scale(n, vec3_dot(forward, n))));
  vec3_t side = vec3_normalize(vec3_cross(n, forward)); /* right-hand side */
  vec3_t velocity = physics_body_velocity_at_point(body, wheel->contact_point);
  float v_forward = vec3_dot(velocity, forward);
  float v_side = vec3_dot(velocity, side);

  bool locked = wheel->handbraked && vehicle->handbrake;
  float grip_factor = locked ? p->handbrake_grip_factor : 1.0f;
  float available =
      p->tyre_friction * wheel->surface_friction * wheel->load * grip_factor;

  /* Longitudinal: drive, brakes, rolling resistance. Braking is capped at
   * what would stop this wheel's share of the car within the step. */
  float share = p->mass / VEHICLE_WHEEL_COUNT;
  float stopping = share * fabsf(v_forward) / dt;
  float braking = p->brake_force / VEHICLE_WHEEL_COUNT * brake;
  if (locked) braking += p->handbrake_force / 2.0f;
  braking += p->rolling_resistance * wheel->load;
  if (braking > stopping) braking = stopping;
  float f_long =
      (wheel->driven ? drive_force : 0.0f) - signf(v_forward) * braking;

  /* Lateral: linear in slip angle up to the friction circle. */
  float slip = atan2f(v_side, fmaxf(fabsf(v_forward), 1.0f));
  wheel->slip_angle = slip;
  float f_side = -p->tyre_lateral_stiffness * slip * wheel->load;
  float lateral_stopping = share * fabsf(v_side) / dt;
  if (fabsf(f_side) > lateral_stopping)
    f_side = -signf(v_side) * lateral_stopping;

  float demanded = sqrtf(f_long * f_long + f_side * f_side);
  wheel->grip_use = available > 0 ? demanded / available : 1.0f;
  if (demanded > available && demanded > 0) {
    float scale = available / demanded;
    f_long *= scale;
    f_side *= scale;
  }
  wheel->skidding =
      (wheel->grip_use > 1.8f && fabsf(v_forward) + fabsf(v_side) > 1.0f) ||
      (fabsf(slip) > SKID_SLIP_ANGLE && fabsf(v_forward) > 4.0f);
  vec3_t force =
      vec3_add(vec3_scale(forward, f_long), vec3_scale(side, f_side));
  physics_apply_force_at_world_point(
      world, vehicle->body, force,
      vec3_add_scaled(wheel->contact_point, up, p->roll_centre_lift));

  wheel->spin = v_forward / p->wheel_radius;
  if (locked && fabsf(v_forward) > 0.5f) wheel->spin = 0;
  wheel->rotation += wheel->spin * dt;
}

void vehicle_update(vehicle_t* vehicle, physics_world_t* world,
                    driving_input_t input, float dt) {
  const vehicle_params_t* p = &vehicle->params;
  rigid_body_t* body = physics_body(world, vehicle->body);
  if (!body) return;
  vehicle->forward_speed =
      vec3_dot(body->velocity, quat_forward(body->rotation));
  vehicle->throttle = clampf(input.throttle, 0, 1);
  vehicle->brake = clampf(input.brake, 0, 1);
  vehicle->handbrake = input.handbrake;
  vehicle->steering =
      steering_filter_update(&vehicle->steering_filter, input.steering,
                             fabsf(vehicle->forward_speed), dt);
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++)
    vehicle->wheels[i].steer_angle =
        vehicle->wheels[i].steerable ? -vehicle->steering * p->max_steer_angle
                                     : 0.0f;

  update_engine(vehicle);
  update_gearbox(vehicle, vehicle->throttle, vehicle->brake, dt);
  float throttle = vehicle->throttle, brake = vehicle->brake;
  if (vehicle->gear == VEHICLE_GEAR_REVERSE) {
    /* In reverse the brake pedal backs the car up and the throttle brakes. */
    throttle = vehicle->brake;
    brake = vehicle->throttle;
  }
  float drive = drive_force_per_wheel(vehicle, throttle);

  vehicle->skid_intensity = 0;
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
    update_wheel(vehicle, world, body, i, drive, brake, dt);
    if (vehicle->wheels[i].skidding)
      vehicle->skid_intensity = fmaxf(
          vehicle->skid_intensity,
          clampf((vehicle->wheels[i].grip_use - 1.3f) / 0.8f, 0.0f, 1.0f));
  }

  float speed = vec3_length(body->velocity);
  if (speed > 0.01f)
    physics_apply_force(
        world, vehicle->body,
        vec3_scale(body->velocity, -p->drag_coefficient * speed));

  vehicle->impact_impulse = 0;
  for (size_t c = 0; c < physics_contact_count(world); c++) {
    const contact_event_t* contact = physics_contact_at(world, c);
    if (contact->body.id == vehicle->body.id &&
        contact->impulse > vehicle->impact_impulse)
      vehicle->impact_impulse = contact->impulse;
  }
  /* A hard impact dents it; a scrape or a bump does not. Roughly a third of
   * a wreck for a crash at forty kilometres an hour into a wall. */
  /* Impulse grows with mass, so the thresholds are for a 1200 kg car and
   * scale up for a lorry: the same crash dents it as much. */
  const float mass_scale = fmaxf(p->mass / 1200.0f, 1.0f);
  const float dent_floor = 1500.0f * mass_scale,
              dent_scale = 12000.0f * mass_scale;
  if (vehicle->impact_impulse > dent_floor) {
    vehicle->damage += (vehicle->impact_impulse - dent_floor) / dent_scale;
    if (vehicle->damage > 1.0f) vehicle->damage = 1.0f;
  }
}

/* ---- telemetry ---- */

float vehicle_forward_speed(const vehicle_t* vehicle) {
  return vehicle->forward_speed;
}
float vehicle_power_factor(const vehicle_t* vehicle) {
  return 1.0f - 0.45f * vehicle->damage;
}

float vehicle_speed_kmh(const vehicle_t* vehicle) {
  return vehicle->forward_speed * 3.6f;
}
float vehicle_rpm(const vehicle_t* vehicle) { return vehicle->rpm; }
int vehicle_gear(const vehicle_t* vehicle) { return vehicle->gear; }
float vehicle_skid_intensity(const vehicle_t* vehicle) {
  return vehicle->skid_intensity;
}

transform_t vehicle_wheel_transform(const vehicle_t* vehicle,
                                    transform_t chassis, int index) {
  const vehicle_params_t* p = &vehicle->params;
  const wheel_state_t* wheel = &vehicle->wheels[index];
  vec3_t local_hub =
      vec3_add(wheel->attach_local,
               vec3(0, -(p->suspension_rest_length - wheel->compression), 0));
  transform_t t = transform_identity();
  t.position = transform_point(chassis, local_hub);
  quat_t steer = quat_from_yaw(wheel->steer_angle);
  quat_t spin = quat_from_axis_angle(vec3(1, 0, 0), -wheel->rotation);
  t.rotation = quat_multiply(chassis.rotation, quat_multiply(steer, spin));
  return t;
}
