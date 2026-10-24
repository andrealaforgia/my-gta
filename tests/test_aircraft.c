#include <assert.h>
#include <math.h>

#include "aircraft.h"
#include "test_groups.h"
#include "test_support.h"

#define DT (1.0f / 120.0f)

static aircraft_controls_t controls(float pitch, float roll, float power) {
  aircraft_controls_t c = {pitch, roll, power, 0};
  return c;
}

/* Full power down the runway, the nose coming up at speed. */
static aircraft_t plane_taking_off(float seconds) {
  aircraft_t plane = aircraft_create(AIRCRAFT_PLANE, vec3(0, 0, 0), 0);
  for (float t = 0; t < seconds; t += DT)
    aircraft_step(&plane, controls(t > 10.0f ? 0.6f : 0, 0, 1), 0, DT);
  return plane;
}

static void a_plane_waits_on_the_runway_until_given_power(void) {
  aircraft_t plane = aircraft_create(AIRCRAFT_PLANE, vec3(5, 0, 5), 1.0f);
  for (int i = 0; i < 240; i++)
    assert(aircraft_step(&plane, controls(0, 0, 0), 0, DT) == AIRCRAFT_FLYING);
  assert(plane.on_ground && plane.speed == 0);
  assert(plane.position.x == 5 && plane.position.z == 5);
}

static void a_plane_takes_off_climbs_and_goes_where_its_nose_points(void) {
  aircraft_t plane = plane_taking_off(40.0f);
  assert(!plane.on_ground && plane.position.y > 30.0f);
  assert(plane.speed > 30.0f);
  /* Yaw zero is nose to -Z. */
  assert(plane.position.z < -300.0f && fabsf(plane.position.x) < 1.0f);
}

static void a_plane_held_nose_up_with_no_power_stalls_and_sinks(void) {
  aircraft_t plane = plane_taking_off(40.0f);
  float peak = plane.position.y;
  for (float t = 0; t < 40.0f; t += DT) {
    aircraft_step(&plane, controls(1, 0, -1), 0, DT);
    peak = fmaxf(peak, plane.position.y);
  }
  assert(plane.position.y < peak - 20.0f || plane.on_ground);
}

static void banking_turns_the_plane_the_way_it_leans(void) {
  aircraft_t plane = plane_taking_off(40.0f);
  float yaw = plane.yaw;
  for (float t = 0; t < 4.0f; t += DT)
    aircraft_step(&plane, controls(0, 1, 0), 0, DT);
  /* Right bank, so the heading turns clockwise seen from above: yaw falls. */
  assert(plane.yaw < yaw - 0.2f && plane.roll > 0.3f);
}

static void a_hard_landing_is_a_crash_and_a_gentle_one_is_not(void) {
  aircraft_t diving = plane_taking_off(40.0f);
  diving.position.y = 80.0f;
  aircraft_status_t status = AIRCRAFT_FLYING;
  for (float t = 0; t < 30.0f && status == AIRCRAFT_FLYING; t += DT)
    status = aircraft_step(&diving, controls(-1, 0, 0), 0, DT);
  assert(status == AIRCRAFT_CRASHED);
  /* Power off, nose steady: it comes down gently and rolls to a stop. */
  aircraft_t gliding = plane_taking_off(40.0f);
  gliding.position.y = 25.0f;
  status = AIRCRAFT_FLYING;
  for (float t = 0; t < 120.0f && !gliding.on_ground; t += DT)
    status = aircraft_step(&gliding, controls(0, 0, -0.3f), 0, DT);
  assert(status == AIRCRAFT_FLYING && gliding.on_ground);
}

static void a_helicopter_needs_the_rotor_to_spin_up_before_it_lifts(void) {
  aircraft_t heli = aircraft_create(AIRCRAFT_HELICOPTER, vec3(0, 0, 0), 0);
  for (int i = 0; i < 120; i++) aircraft_step(&heli, controls(0, 0, 1), 0, DT);
  assert(heli.position.y < 0.01f && heli.rotor_speed < 0.9f);
  for (float t = 0; t < 4.0f; t += DT)
    aircraft_step(&heli, controls(0, 0, 1), 0, DT);
  assert(heli.position.y > 5.0f && !heli.on_ground);
}

static void a_helicopter_hovers_flies_forward_and_lands(void) {
  aircraft_t heli = aircraft_create(AIRCRAFT_HELICOPTER, vec3(0, 0, 0), 0);
  for (float t = 0; t < 7.0f; t += DT)
    aircraft_step(&heli, controls(0, 0, t < 6.0f ? 1.0f : 0.0f), 0, DT);
  float hover = heli.position.y;
  for (int i = 0; i < 240; i++) aircraft_step(&heli, controls(0, 0, 0), 0, DT);
  assert(fabsf(heli.position.y - hover) < 0.5f);
  for (float t = 0; t < 5.0f; t += DT)
    aircraft_step(&heli, controls(-1, 0, 0), 0, DT);
  assert(heli.position.z < -50.0f && heli.speed > 25.0f);
  aircraft_status_t status = AIRCRAFT_FLYING;
  for (float t = 0; t < 40.0f && !heli.on_ground; t += DT)
    status = aircraft_step(&heli, controls(0, 0, -0.3f), 0, DT);
  assert(status == AIRCRAFT_FLYING && heli.on_ground);
}

/* With nobody flying it, a plane in the air noses over and hits the ground. */
static void a_plane_left_without_a_pilot_dives_and_crashes(void) {
  aircraft_t plane = plane_taking_off(40.0f);
  assert(!plane.on_ground);
  plane.position.y = 150.0f; /* well up */
  plane.doomed = true;
  aircraft_status_t status = AIRCRAFT_FLYING;
  float t = 0;
  while (status == AIRCRAFT_FLYING && t < 30.0f) {
    status = aircraft_step(&plane, controls(0, 0, 0), 0, DT);
    t += DT;
  }
  assert(status == AIRCRAFT_CRASHED && t < 20.0f);
}

/* A helicopter loses its lift and falls hard. */
static void a_helicopter_left_without_a_pilot_falls_and_crashes(void) {
  aircraft_t heli = aircraft_create(AIRCRAFT_HELICOPTER, vec3(0, 0, 0), 0);
  for (float t = 0; t < 14.0f; t += DT)
    aircraft_step(&heli, controls(0, 0, 1), 0, DT);
  assert(heli.position.y > 40.0f);
  heli.doomed = true;
  aircraft_status_t status = AIRCRAFT_FLYING;
  float t = 0;
  while (status == AIRCRAFT_FLYING && t < 30.0f) {
    status = aircraft_step(&heli, controls(0, 0, 0), 0, DT);
    t += DT;
  }
  assert(status == AIRCRAFT_CRASHED);
}

static void keys_become_flying_controls(void) {
  driving_input_t forward = {1, 0, -1, true, false};
  aircraft_controls_t c = aircraft_controls_from_input(forward);
  assert(c.pitch < 0 && c.roll < 0 && c.power_change > 0);
  driving_input_t back = {0, 1, 0, false, true};
  c = aircraft_controls_from_input(back);
  assert(c.pitch > 0 && c.power_change < 0 && c.brake > 0);
}

/* Metres of runway a fixed-wing aircraft uses before its wheels leave it,
 * at full power, the nose coming up as soon as it has the speed. */
static float run_to_lift_off(aircraft_kind_t kind) {
  aircraft_t craft = aircraft_create(kind, vec3(0, 0, 0), 0);
  for (int i = 0; i < 120 * 60 && craft.on_ground; i++)
    aircraft_step(&craft, controls(craft.speed > 36.0f ? 0.6f : 0, 0, 1), 0,
                  DT);
  assert(!craft.on_ground);
  return -craft.position.z;
}

static void every_aeroplane_gets_off_a_runway_and_big_ones_need_more(void) {
  float plane = run_to_lift_off(AIRCRAFT_PLANE);
  float jet = run_to_lift_off(AIRCRAFT_JET);
  float airliner = run_to_lift_off(AIRCRAFT_AIRLINER);
  assert(plane < 400.0f && jet < 400.0f && airliner < 850.0f);
  assert(jet < plane && plane < airliner);
}

static void the_jet_outruns_the_plane_and_the_airliner_too(void) {
  /* Climbing away at full power for a minute. */
  float speed[3];
  const aircraft_kind_t kinds[3] = {AIRCRAFT_PLANE, AIRCRAFT_JET,
                                    AIRCRAFT_AIRLINER};
  for (int k = 0; k < 3; k++) {
    aircraft_t craft = aircraft_create(kinds[k], vec3(0, 0, 0), 0);
    for (int i = 0; i < 120 * 60; i++) {
      float pitch = craft.speed > 36.0f && craft.position.y < 400.0f ? 0.3f : 0;
      aircraft_step(&craft, controls(pitch, 0, 1), 0, DT);
    }
    speed[k] = craft.speed;
  }
  assert(speed[1] > 2.0f * speed[0]);
  assert(speed[2] > speed[0]);
}

static void only_the_jet_and_the_gunship_carry_missiles(void) {
  assert(aircraft_has_missiles(AIRCRAFT_JET));
  assert(aircraft_has_missiles(AIRCRAFT_GUNSHIP));
  assert(!aircraft_has_missiles(AIRCRAFT_PLANE));
  assert(!aircraft_has_missiles(AIRCRAFT_HELICOPTER));
  assert(!aircraft_has_missiles(AIRCRAFT_AIRLINER));
  assert(aircraft_is_fixed_wing(AIRCRAFT_AIRLINER) &&
         aircraft_is_fixed_wing(AIRCRAFT_JET));
  assert(!aircraft_is_fixed_wing(AIRCRAFT_GUNSHIP));
}

static void the_gunship_flies_faster_than_the_little_helicopter(void) {
  float speed[2];
  const aircraft_kind_t kinds[2] = {AIRCRAFT_HELICOPTER, AIRCRAFT_GUNSHIP};
  for (int k = 0; k < 2; k++) {
    aircraft_t craft = aircraft_create(kinds[k], vec3(0, 0, 0), 0);
    for (int i = 0; i < 120 * 25; i++) {
      bool lifting = i < 120 * 10;
      aircraft_step(&craft, controls(lifting ? 0 : -1, 0, lifting ? 1 : 0), 0,
                    DT);
    }
    speed[k] = craft.speed;
  }
  assert(speed[1] > speed[0] * 1.2f);
}

void run_aircraft_tests(void) {
  RUN(a_plane_waits_on_the_runway_until_given_power);
  RUN(a_plane_takes_off_climbs_and_goes_where_its_nose_points);
  RUN(a_plane_held_nose_up_with_no_power_stalls_and_sinks);
  RUN(banking_turns_the_plane_the_way_it_leans);
  RUN(a_hard_landing_is_a_crash_and_a_gentle_one_is_not);
  RUN(a_helicopter_needs_the_rotor_to_spin_up_before_it_lifts);
  RUN(a_helicopter_hovers_flies_forward_and_lands);
  RUN(a_plane_left_without_a_pilot_dives_and_crashes);
  RUN(a_helicopter_left_without_a_pilot_falls_and_crashes);
  RUN(every_aeroplane_gets_off_a_runway_and_big_ones_need_more);
  RUN(the_jet_outruns_the_plane_and_the_airliner_too);
  RUN(only_the_jet_and_the_gunship_carry_missiles);
  RUN(the_gunship_flies_faster_than_the_little_helicopter);
  RUN(keys_become_flying_controls);
}
