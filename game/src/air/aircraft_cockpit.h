/**
 * @file aircraft_cockpit.h
 * @brief What the pilot sees from the seat: the panel, the windscreen frame,
 * the dials, and the yoke or stick that follows the controls. One flight deck
 * per kind of aircraft, built in the aircraft's own frame.
 */
#ifndef GAME_SRC_AIR_AIRCRAFT_COCKPIT_H_
#define GAME_SRC_AIR_AIRCRAFT_COCKPIT_H_

#include "aircraft.h"
#include "aircraft_visual.h"
#include "renderer.h"

/* Where the pilot's eyes are, in the aircraft's own frame. */
vec3_t aircraft_cockpit_eye(aircraft_kind_t kind);
/* Draws the flight deck about the aircraft: the panel, the round dials
 * (airspeed, the blue and brown horizon, altitude over the ground, compass),
 * the landing-gear lights and lever, the throttle and the yoke or stick.
 * `stick` is what the pilot is asking for and moves them; `height` is the
 * metres above the ground. */
void aircraft_visual_draw_cockpit(const aircraft_visual_t* visual,
                                  renderer_t* renderer, const aircraft_t* a,
                                  aircraft_controls_t stick, float height);
/* Everything that turns outside the windscreen: the propeller or rotors,
 * which are all of the aircraft that is drawn from inside. */
void aircraft_visual_draw_spinners(const aircraft_visual_t* visual,
                                   renderer_t* renderer, const aircraft_t* a);
/* A gunsight ahead of the nose of an aircraft that carries missiles. */
void aircraft_visual_draw_sight(renderer_t* renderer, const aircraft_t* a);

/* Cockpit meshes, made and released with the rest of the visuals. */
void aircraft_cockpit_create(aircraft_visual_t* visual, renderer_t* renderer);
void aircraft_cockpit_destroy(aircraft_visual_t* visual, renderer_t* renderer);

#endif  // GAME_SRC_AIR_AIRCRAFT_COCKPIT_H_
