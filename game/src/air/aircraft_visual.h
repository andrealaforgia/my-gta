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
} aircraft_visual_t;

aircraft_visual_t aircraft_visual_create(renderer_t* renderer);
void aircraft_visual_destroy(aircraft_visual_t* visual, renderer_t* renderer);
/* `variant` picks the paint. */
void aircraft_visual_draw(const aircraft_visual_t* visual, renderer_t* renderer,
                          const aircraft_t* aircraft, int variant);

#endif  // GAME_SRC_AIR_AIRCRAFT_VISUAL_H_
