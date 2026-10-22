#include "aircraft.h"

#include <math.h>

#define GRAVITY 9.8f
#define STALL_SPEED 24.0f
#define LIFT_OFF_SPEED 34.0f
#define PLANE_THRUST 3.6f
#define PLANE_DRAG 0.0007f
#define PITCH_LIMIT 0.45f
#define HARD_LANDING 7.0f

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/* Eases `value` towards `target`, at `rate` of the gap a second. */
static float ease(float value, float target, float rate, float dt) {
  return value + (target - value) * fminf(1.0f, rate * dt);
}

aircraft_t aircraft_create(aircraft_kind_t kind, vec3_t position, float yaw) {
  aircraft_t a = {0};
  a.kind = kind;
  a.position = position;
  a.yaw = yaw;
  a.on_ground = true;
  return a;
}

aircraft_controls_t aircraft_controls_from_input(driving_input_t input) {
  aircraft_controls_t c = {
      input.brake - input.throttle, input.steering,
      (input.handbrake ? 1.0f : 0.0f) - (input.lower ? 1.0f : 0.0f), 0.0f};
  /* The wheel brakes: the throttle closed and a key held that is not a
   * climb. Space is a power key, so braking is Ctrl on the ground. */
  c.brake = input.lower ? 1.0f : 0.0f;
  return c;
}

vec3_t aircraft_heading(const aircraft_t* a) {
  return vec3(-sinf(a->yaw), 0, -cosf(a->yaw));
}

vec3_t aircraft_velocity(const aircraft_t* a) {
  vec3_t h = aircraft_heading(a);
  float flat = a->kind == AIRCRAFT_PLANE ? a->speed * cosf(a->path) : a->speed;
  return vec3(h.x * flat, a->climb, h.z * flat);
}

transform_t aircraft_transform(const aircraft_t* a) {
  quat_t turn = quat_multiply(
      quat_from_yaw(a->yaw),
      quat_multiply(quat_from_axis_angle(vec3(1, 0, 0), a->pitch),
                    quat_from_axis_angle(vec3(0, 0, -1), a->roll)));
  return transform_at(a->position, turn);
}

float aircraft_height(aircraft_kind_t kind) {
  return kind == AIRCRAFT_PLANE ? 3.0f : 3.6f;
}

float aircraft_radius(aircraft_kind_t kind) {
  return kind == AIRCRAFT_PLANE ? 6.0f : 6.5f;
}

static void move(aircraft_t* a, float flat_speed, float dt) {
  vec3_t h = aircraft_heading(a);
  a->position.x += h.x * flat_speed * dt;
  a->position.z += h.z * flat_speed * dt;
  a->position.y += a->climb * dt;
}

static aircraft_status_t step_plane(aircraft_t* a, aircraft_controls_t c,
                                    float ground, float dt) {
  a->power = clampf(a->power + c.power_change * 0.5f * dt, 0.0f, 1.0f);
  a->rotor += (4.0f + 50.0f * a->power) * dt;
  a->rotor_speed = 1.0f;
  float thrust = a->power * PLANE_THRUST;
  float drag = PLANE_DRAG * a->speed * a->speed;
  if (a->on_ground) {
    float rolling = a->speed > 0.2f ? 0.3f + (c.brake > 0 ? 7.0f : 0.0f) : 0.0f;
    if (a->power <= 0.01f && a->speed > 0.2f) rolling += 0.6f;
    a->speed = fmaxf(0.0f, a->speed + (thrust - drag - rolling) * dt);
    float turn = -c.roll * 0.55f * fminf(1.0f, a->speed / 6.0f);
    a->yaw += turn * dt;
    a->yaw_rate = turn;
    /* The nose comes up with the stick once there is speed to lift on. */
    if (a->speed > 26.0f && c.pitch > 0)
      a->pitch = fminf(a->pitch + c.pitch * 0.5f * dt, 0.35f);
    else
      a->pitch = ease(a->pitch, 0.0f, 2.0f, dt);
    a->roll = 0;
    a->path = 0;
    a->climb = 0;
    move(a, a->speed, dt);
    a->position.y = ground;
    float lift = a->speed / LIFT_OFF_SPEED;
    if (lift * lift > 1.0f && a->pitch > 0.06f) {
      a->on_ground = false;
      a->path = a->pitch * 0.5f;
    }
    return AIRCRAFT_FLYING;
  }
  float airflow = clampf((a->speed - STALL_SPEED) / 10.0f, 0.0f, 1.0f);
  a->pitch += c.pitch * 0.8f * dt;
  if (c.pitch == 0) a->pitch = ease(a->pitch, 0.04f, 0.4f, dt);
  /* Without the speed to hold it up the nose drops. */
  a->pitch -= (1.0f - airflow) * 0.5f * dt;
  a->pitch = clampf(a->pitch, -PITCH_LIMIT, PITCH_LIMIT);
  a->roll = ease(a->roll, c.roll * 0.9f, 2.5f, dt);
  float target = a->pitch * airflow - 0.5f * (1.0f - airflow);
  a->path = ease(a->path, target, 1.5f, dt);
  a->speed =
      fmaxf(0.0f, a->speed + (thrust - drag - GRAVITY * sinf(a->path)) * dt);
  float turn = -tanf(a->roll) * GRAVITY / fmaxf(a->speed, 25.0f);
  a->yaw_rate = clampf(turn, -1.0f, 1.0f);
  a->yaw += a->yaw_rate * dt;
  a->climb = a->speed * sinf(a->path);
  move(a, a->speed * cosf(a->path), dt);
  if (a->position.y <= ground) {
    float sink = -a->climb;
    bool hard = sink > HARD_LANDING || fabsf(a->roll) > 0.4f || a->path < -0.3f;
    a->position.y = ground;
    if (hard) return AIRCRAFT_CRASHED;
    a->on_ground = true;
    a->roll = 0;
    a->path = 0;
    a->climb = 0;
    a->pitch = fmaxf(a->pitch, 0.0f);
  }
  return AIRCRAFT_FLYING;
}

static aircraft_status_t step_helicopter(aircraft_t* a, aircraft_controls_t c,
                                         float ground, float dt) {
  a->rotor_speed = fminf(1.0f, a->rotor_speed + 0.3f * dt);
  a->rotor += a->rotor_speed * 38.0f * dt;
  float height = a->position.y - ground;
  bool lifting = a->rotor_speed >= 0.9f;
  float want_climb =
      lifting ? c.power_change * (c.power_change > 0 ? 10.0f : 8.0f) : 0.0f;
  a->climb = ease(a->climb, want_climb, 2.2f, dt);
  /* Forward on W (nose down), back on S, slower; none while on the ground. */
  float want = c.pitch < 0 ? -c.pitch * 42.0f : -c.pitch * 14.0f;
  if (height < 1.0f) want = 0;
  a->speed = ease(a->speed, want, 0.8f, dt);
  float turn = -c.roll * 1.1f * (height < 0.5f ? 0.3f : 1.0f);
  a->yaw_rate = ease(a->yaw_rate, turn, 3.0f, dt);
  a->yaw += a->yaw_rate * dt;
  a->pitch = ease(a->pitch, -a->speed / 42.0f * 0.3f, 2.0f, dt);
  a->roll =
      ease(a->roll, c.roll * 0.3f * fminf(1.0f, a->speed / 8.0f), 2.0f, dt);
  move(a, a->speed, dt);
  a->on_ground = false;
  if (a->position.y <= ground) {
    bool hard = -a->climb > HARD_LANDING + 3.0f;
    a->position.y = ground;
    if (hard) return AIRCRAFT_CRASHED;
    a->climb = fmaxf(a->climb, 0.0f);
    a->on_ground = true;
  }
  return AIRCRAFT_FLYING;
}

aircraft_status_t aircraft_step(aircraft_t* aircraft,
                                aircraft_controls_t controls, float ground,
                                float dt) {
  controls.pitch = clampf(controls.pitch, -1.0f, 1.0f);
  controls.roll = clampf(controls.roll, -1.0f, 1.0f);
  controls.power_change = clampf(controls.power_change, -1.0f, 1.0f);
  return aircraft->kind == AIRCRAFT_PLANE
             ? step_plane(aircraft, controls, ground, dt)
             : step_helicopter(aircraft, controls, ground, dt);
}
