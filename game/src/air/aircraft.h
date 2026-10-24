/**
 * @file aircraft.h
 * @brief An aeroplane or a helicopter and how it flies: a simple arcade
 * model with no physics world. The caller says how high the ground is under
 * it; what it flies into is the caller's business. Nose towards -Z when yaw
 * is zero, as everywhere else.
 */
#ifndef GAME_SRC_AIR_AIRCRAFT_H_
#define GAME_SRC_AIR_AIRCRAFT_H_

#include <stdbool.h>

#include "city_package.h"
#include "input.h"
#include "transform.h"
#include "vec3.h"

typedef struct {
  aircraft_kind_t kind;
  vec3_t position; /* the belly, where the wheels or skids touch */
  float yaw, pitch, roll;
  float speed;       /* along the flight path; a helicopter's, forward */
  float climb;       /* metres a second, up */
  float path;        /* an aeroplane's flight path angle, radians up */
  float power;       /* an aeroplane's throttle, 0..1 */
  float rotor;       /* propeller or rotor angle, radians */
  float rotor_speed; /* 0 stopped .. 1 flying speed */
  float yaw_rate;
  bool on_ground;
  bool gear_down; /* the wheels are asked for down (an aeroplane's) */
  float gear;     /* 1 down and locked .. 0 folded away; takes a few seconds */
  bool doomed;    /* left with nobody at the controls: it dives and crashes */
} aircraft_t;

/* What the pilot asks for, each -1 to 1: pitch positive is nose up, roll
 * positive is to the right; power_change raises or lowers the throttle (the
 * helicopter's climb); brake holds it on the ground. */
typedef struct {
  float pitch, roll, power_change, brake;
} aircraft_controls_t;

typedef enum { AIRCRAFT_FLYING, AIRCRAFT_CRASHED } aircraft_status_t;

/* Aeroplanes of every size run on wings; helicopters and the gunship on
 * rotors. The jet and the gunship carry missiles. */
bool aircraft_is_fixed_wing(aircraft_kind_t kind);
bool aircraft_has_missiles(aircraft_kind_t kind);
/* Raises or lowers the wheels of an aeroplane that is in the air. False if it
 * cannot: a helicopter, or an aeroplane standing on its wheels. */
bool aircraft_toggle_gear(aircraft_t* aircraft);
aircraft_t aircraft_create(aircraft_kind_t kind, vec3_t position, float yaw);
/* The keys and pad as controls: W pushes the nose down and S pulls it up,
 * A and D bank, Shift or Space add power, Ctrl takes it off. */
aircraft_controls_t aircraft_controls_from_input(driving_input_t input);
/* One step; `ground` is the height of the ground under the aircraft. A hard
 * landing is a crash. */
aircraft_status_t aircraft_step(aircraft_t* aircraft,
                                aircraft_controls_t controls, float ground,
                                float dt);
vec3_t aircraft_heading(const aircraft_t* aircraft); /* level, a unit vector */
vec3_t aircraft_velocity(const aircraft_t* aircraft);
transform_t aircraft_transform(const aircraft_t* aircraft);
/* Metres from the belly up to the highest point, and the radius of the
 * smallest sphere that holds it, for what it may bump into. */
float aircraft_height(aircraft_kind_t kind);
float aircraft_radius(aircraft_kind_t kind);

#endif  // GAME_SRC_AIR_AIRCRAFT_H_
