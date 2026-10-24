/**
 * @file aircraft_visual.h
 * @brief The shapes of the light aeroplane and the helicopter, built from
 * boxes and ellipsoids, and how they are drawn: paint, glass, trim, and a
 * propeller or rotor turned to its angle.
 */
#ifndef GAME_SRC_AIR_AIRCRAFT_VISUAL_H_
#define GAME_SRC_AIR_AIRCRAFT_VISUAL_H_

#include "aircraft.h"
#include "renderer.h"

typedef struct {
  mesh_handle_t plane_paint, plane_trim, plane_glass, plane_prop;
  mesh_handle_t heli_paint, heli_trim, heli_glass, heli_rotor, heli_tail_rotor;
  mesh_handle_t canopy;                              /* a parachute */
  mesh_handle_t plane_gear, airliner_gear, jet_gear; /* wheels and legs */
  mesh_handle_t airliner_paint, airliner_trim, airliner_glass, airliner_accent;
  mesh_handle_t jet_paint, jet_trim, jet_glass, jet_flame;
  mesh_handle_t gunship_paint, gunship_trim, gunship_glass, gunship_rotor;
  mesh_handle_t missile, missile_flame;
  /* Flight decks, a kind each: the panel, the lit dials, the yoke or stick. */
  mesh_handle_t cockpit_dark[AIRCRAFT_KIND_COUNT];
  mesh_handle_t cockpit_lit[AIRCRAFT_KIND_COUNT];
  mesh_handle_t cockpit_stick[AIRCRAFT_KIND_COUNT];
  /* Instruments, shared by every flight deck. */
  mesh_handle_t gauge_body, gauge_ticks_speed, gauge_ticks_altitude,
      gauge_needle, gauge_needle_short, compass_card, compass_north, lubber,
      ball_sky, ball_ground, ball_symbol, light_disc, lever, bar;
} aircraft_visual_t;

aircraft_visual_t aircraft_visual_create(renderer_t* renderer);
void aircraft_visual_destroy(aircraft_visual_t* visual, renderer_t* renderer);
/* `variant` picks the paint. */
void aircraft_visual_draw(const aircraft_visual_t* visual, renderer_t* renderer,
                          const aircraft_t* aircraft, int variant);

/* A missile in flight at `at`, flying along the unit vector `direction`,
 * with its flame. */
void aircraft_visual_draw_missile(const aircraft_visual_t* visual,
                                  renderer_t* renderer, vec3_t at,
                                  vec3_t direction);

/* A parachute over somebody whose feet are at `feet`, `open` from 0 (packed)
 * to 1 (full), with its lines down to the shoulders; `tilt` swings it to one
 * side (radians, positive to the right) as it sways or banks into a turn. */
void aircraft_visual_draw_canopy(const aircraft_visual_t* visual,
                                 renderer_t* renderer, vec3_t feet, float yaw,
                                 float open, float tilt);

#endif  // GAME_SRC_AIR_AIRCRAFT_VISUAL_H_
