/**
 * @file guide_hud.h
 * @brief Showing the way to the airport: the pillar of light over its gate,
 * the arrow and distance on the screen, the line of the road on the minimap
 * and the map, and the mark at its end.
 */
#ifndef GAME_SRC_STAGE_GUIDE_HUD_H_
#define GAME_SRC_STAGE_GUIDE_HUD_H_

#include <stdbool.h>
#include <stddef.h>

#include "city_map.h"
#include "renderer.h"

#define GUIDE_RING_RADIUS 9.0f

/* Where the guide points: the airport's gate. */
typedef struct {
  bool active;
  float x, y, z;
} guide_t;

typedef struct {
  mesh_handle_t pillar, ring;
} guide_markers_t;

guide_markers_t guide_markers_create(renderer_t* renderer);
void guide_markers_destroy(guide_markers_t* markers, renderer_t* renderer);

/* The pillar and ring at the gate, in the world; `clock` pulses the light. */
void guide_marker_draw(const guide_t* guide, const guide_markers_t* markers,
                       renderer_t* renderer, float clock);

/* The compass arrow towards (aim_x, aim_z) and the distance to the airport.
 * The player is at (x, z) heading (fx, fz), a unit vector. */
void guide_hud_draw(const guide_t* guide, renderer_t* renderer,
                    float screen_width, float screen_height, float x, float z,
                    float fx, float fz, float aim_x, float aim_z);

/* The satnav line, x, z pairs, on the minimap window as for
 * guide_minimap_mark, cut to the window. */
void guide_route_on_minimap(renderer_t* renderer, const city_map_t* map,
                            const float* line, size_t count, float left,
                            float top, float side, float u0, float v0, float u1,
                            float v1, float scale);

/* The same on the full-screen map. */
void guide_route_on_map(renderer_t* renderer, const city_map_t* map,
                        const float* line, size_t count, float left, float top,
                        float side, float scale);

/* A flashing mark on the minimap window at (left, top, side) showing
 * texture coordinates u0..u1, v0..v1 of the map; held at the edge when the
 * destination lies beyond it. */
void guide_minimap_mark(const guide_t* guide, renderer_t* renderer,
                        const city_map_t* map, float left, float top,
                        float side, float u0, float v0, float u1, float v1,
                        float scale, float clock);

/* The same on the full-screen map. */
void guide_map_mark(const guide_t* guide, renderer_t* renderer,
                    const city_map_t* map, float left, float top, float side,
                    float scale, float clock);

#endif  // GAME_SRC_STAGE_GUIDE_HUD_H_
