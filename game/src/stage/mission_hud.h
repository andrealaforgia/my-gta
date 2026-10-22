/**
 * @file mission_hud.h
 * @brief Showing a delivery run: the pillar of light over the destination,
 * the arrow, distance, clock and cash on the screen, and the mark on the
 * minimap and the map.
 */
#ifndef GAME_SRC_STAGE_MISSION_HUD_H_
#define GAME_SRC_STAGE_MISSION_HUD_H_

#include "city_map.h"
#include "mission.h"
#include "renderer.h"

typedef struct {
  mesh_handle_t pillar, ring;
} mission_markers_t;

mission_markers_t mission_markers_create(renderer_t* renderer);
void mission_markers_destroy(mission_markers_t* markers, renderer_t* renderer);

/* The pillar and ring at the destination, in the world. `ground` is the
 * height of the road there; `clock` pulses the light. */
void mission_marker_draw(const mission_t* mission,
                         const mission_markers_t* markers, renderer_t* renderer,
                         float ground, float clock);

/* The compass arrow, the distance, the time left and the cash, and the
 * result of the last run. The car is at (x, z) heading (fx, fz), a unit
 * vector. */
void mission_hud_draw(const mission_t* mission, renderer_t* renderer,
                      float screen_width, float screen_height, float x, float z,
                      float fx, float fz);

/* A flashing mark on the minimap window at (left, top, side) showing
 * texture coordinates u0..u1, v0..v1 of the map; held at the edge when the
 * destination lies beyond it. */
void mission_minimap_mark(const mission_t* mission, renderer_t* renderer,
                          const city_map_t* map, float left, float top,
                          float side, float u0, float v0, float u1, float v1,
                          float scale, float clock);

/* The same on the full-screen map. */
void mission_map_mark(const mission_t* mission, renderer_t* renderer,
                      const city_map_t* map, float left, float top, float side,
                      float scale, float clock);

#endif  // GAME_SRC_STAGE_MISSION_HUD_H_
