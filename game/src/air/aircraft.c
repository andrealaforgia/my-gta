#include "aircraft.h"

#include <math.h>

#define GRAVITY 9.8f
#define HARD_LANDING 7.0f

/* What sets one aeroplane apart from another. */
typedef struct {
  float stall, lift_off;  /* m/s: lift fails below, and the wheels leave */
  float rotate;           /* m/s at which the nose may come up on the runway */
  float thrust, drag;     /* m/s^2 at full power, and per m/s squared */
  float ground_turn;      /* rad/s of steering on the ground */
  float ground_pitch_cap; /* how far the nose rises before take-off */
  float pitch_rate;       /* rad/s of elevator in the air */
  float pitch_limit;      /* radians */
  float roll_rate, roll_max;
  float turn_gain;     /* how hard a banked turn bites */
  float airflow_range; /* m/s over the stall to full authority */
  float power_rate;    /* throttle change a second */
  float hard_landing;  /* m/s of sink that wrecks it */
} wing_t;

static const wing_t wings[AIRCRAFT_KIND_COUNT] = {
    [AIRCRAFT_PLANE] = {24, 34, 26, 3.6f, 0.0007f, 0.55f, 0.35f, 0.8f, 0.45f,
                        2.5f, 0.9f, 1.0f, 10, 0.5f, 7},
    [AIRCRAFT_AIRLINER] = {52, 66, 54, 5.0f, 0.00028f, 0.22f, 0.22f, 0.35f,
                           0.3f, 1.0f, 0.5f, 1.0f, 16, 0.3f, 6},
    [AIRCRAFT_JET] = {38, 52, 40, 13.0f, 0.00019f, 0.6f, 0.4f, 1.2f, 0.6f, 5.0f,
                      1.3f, 2.4f, 10, 0.8f, 8},
};

bool aircraft_is_fixed_wing(aircraft_kind_t kind) {
  return kind == AIRCRAFT_PLANE || kind == AIRCRAFT_AIRLINER ||
         kind == AIRCRAFT_JET;
}

bool aircraft_has_missiles(aircraft_kind_t kind) {
  return kind == AIRCRAFT_JET || kind == AIRCRAFT_GUNSHIP;
}

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
  a.gear_down = true;
  a.gear = 1.0f;
  return a;
}

bool aircraft_toggle_gear(aircraft_t* a) {
  if (!aircraft_is_fixed_wing(a->kind) || a->on_ground) return false;
  a->gear_down = !a->gear_down;
  return true;
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
  float flat =
      aircraft_is_fixed_wing(a->kind) ? a->speed * cosf(a->path) : a->speed;
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
  static const float heights[AIRCRAFT_KIND_COUNT] = {3.0f, 3.6f, 19.0f, 5.0f,
                                                     5.0f};
  return heights[kind];
}

float aircraft_radius(aircraft_kind_t kind) {
  static const float radii[AIRCRAFT_KIND_COUNT] = {6.0f, 6.5f, 38.0f, 10.0f,
                                                   12.0f};
  return radii[kind];
}

static void advance(aircraft_t* a, float flat_speed, float dt) {
  vec3_t h = aircraft_heading(a);
  a->position.x += h.x * flat_speed * dt;
  a->position.z += h.z * flat_speed * dt;
  a->position.y += a->climb * dt;
}

static aircraft_status_t step_plane(aircraft_t* a, aircraft_controls_t c,
                                    float ground, float dt) {
  const wing_t* w = &wings[a->kind];
  a->power = clampf(a->power + c.power_change * w->power_rate * dt, 0.0f, 1.0f);
  a->rotor += (4.0f + 50.0f * a->power) * dt;
  a->rotor_speed = 1.0f;
  float thrust = a->power * w->thrust;
  /* Wheels in the wind cost a sixth of the drag. */
  a->gear = clampf(a->gear + (a->gear_down ? dt : -dt) / 4.0f, 0.0f, 1.0f);
  float drag = w->drag * (0.85f + 0.15f * a->gear) * a->speed * a->speed;
  if (a->on_ground) {
    float rolling = a->speed > 0.2f ? 0.3f + (c.brake > 0 ? 7.0f : 0.0f) : 0.0f;
    if (a->power <= 0.01f && a->speed > 0.2f) rolling += 0.6f;
    a->speed = fmaxf(0.0f, a->speed + (thrust - drag - rolling) * dt);
    float turn = -c.roll * w->ground_turn * fminf(1.0f, a->speed / 6.0f);
    a->yaw += turn * dt;
    a->yaw_rate = turn;
    /* The nose comes up with the stick once there is speed to lift on. */
    if (a->speed > w->rotate && c.pitch > 0)
      a->pitch = fminf(a->pitch + c.pitch * 0.5f * dt, w->ground_pitch_cap);
    else
      a->pitch = ease(a->pitch, 0.0f, 2.0f, dt);
    a->roll = 0;
    a->path = 0;
    a->climb = 0;
    advance(a, a->speed, dt);
    a->position.y = ground;
    float lift = a->speed / w->lift_off;
    if (lift * lift > 1.0f && a->pitch > 0.06f) {
      a->on_ground = false;
      a->path = a->pitch * 0.5f;
    }
    return AIRCRAFT_FLYING;
  }
  float airflow = clampf((a->speed - w->stall) / w->airflow_range, 0.0f, 1.0f);
  a->pitch += c.pitch * w->pitch_rate * dt;
  if (c.pitch == 0) a->pitch = ease(a->pitch, 0.04f, 0.4f, dt);
  /* Without the speed to hold it up the nose drops. */
  a->pitch -= (1.0f - airflow) * 0.5f * dt;
  a->pitch = clampf(a->pitch, -w->pitch_limit, w->pitch_limit);
  a->roll = ease(a->roll, c.roll * w->roll_max, w->roll_rate, dt);
  float target = a->pitch * airflow - 0.5f * (1.0f - airflow);
  a->path = ease(a->path, target, 1.5f, dt);
  a->speed =
      fmaxf(0.0f, a->speed + (thrust - drag - GRAVITY * sinf(a->path)) * dt);
  float turn = -tanf(a->roll) * GRAVITY * w->turn_gain / fmaxf(a->speed, 25.0f);
  a->yaw_rate = clampf(turn, -1.0f, 1.0f);
  a->yaw += a->yaw_rate * dt;
  a->climb = a->speed * sinf(a->path);
  advance(a, a->speed * cosf(a->path), dt);
  if (a->position.y <= ground) {
    float sink = -a->climb;
    /* Coming down on the belly, with the wheels not locked down, is a wreck. */
    bool hard = a->gear < 0.8f || sink > w->hard_landing ||
                fabsf(a->roll) > 0.4f || a->path < -0.3f;
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
  /* The gunship is faster and climbs harder than the little helicopter. */
  float pace = a->kind == AIRCRAFT_GUNSHIP ? 1.45f : 1.0f;
  a->rotor_speed = fminf(1.0f, a->rotor_speed + 0.3f * dt);
  a->rotor += a->rotor_speed * 38.0f * dt;
  float height = a->position.y - ground;
  bool lifting = a->rotor_speed >= 0.9f;
  float want_climb =
      lifting ? c.power_change * (c.power_change > 0 ? 10.0f : 8.0f) * pace
              : 0.0f;
  /* Nobody at the controls: the rotor loses its bite and it drops. */
  if (a->doomed) want_climb = -22.0f;
  a->climb = ease(a->climb, want_climb, a->doomed ? 1.2f : 2.2f, dt);
  /* Forward on W (nose down), back on S, slower; none while on the ground. */
  float want = c.pitch < 0 ? -c.pitch * 42.0f * pace : -c.pitch * 14.0f;
  if (height < 1.0f) want = 0;
  a->speed = ease(a->speed, want, 0.8f, dt);
  float turn = -c.roll * 1.1f * (height < 0.5f ? 0.3f : 1.0f);
  a->yaw_rate = ease(a->yaw_rate, turn, 3.0f, dt);
  a->yaw += a->yaw_rate * dt;
  a->pitch = ease(a->pitch, -a->speed / (42.0f * pace) * 0.3f, 2.0f, dt);
  a->roll =
      ease(a->roll, c.roll * 0.3f * fminf(1.0f, a->speed / 8.0f), 2.0f, dt);
  advance(a, a->speed, dt);
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
  /* Left to itself, a plane noses over with the engine cut. */
  if (aircraft->doomed) {
    controls.pitch = -1.0f, controls.roll = 0, controls.power_change = -1.0f;
    controls.brake = 0;
  }
  controls.pitch = clampf(controls.pitch, -1.0f, 1.0f);
  controls.roll = clampf(controls.roll, -1.0f, 1.0f);
  controls.power_change = clampf(controls.power_change, -1.0f, 1.0f);
  return aircraft_is_fixed_wing(aircraft->kind)
             ? step_plane(aircraft, controls, ground, dt)
             : step_helicopter(aircraft, controls, ground, dt);
}
