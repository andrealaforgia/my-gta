/**
 * @file guide_hud.h
 * @brief Showing the way to the airport: the arrow and distance on the screen,
 * the line of the road on the minimap and the map, and the mark at its end.
 */
#ifndef GAME_SRC_STAGE_GUIDE_HUD_H_
#define GAME_SRC_STAGE_GUIDE_HUD_H_

#include <stdbool.h>
#include <stddef.h>

#include "city_map.h"
#include "renderer.h"

/* Where the guide points: the airport's gate. */
typedef struct {
  bool active;
  float x, y, z;
} guide_t;

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

/* Police cars after the player, as flashing red and blue dots: `cars` is x, z
 * pairs. On the minimap window, as for guide_minimap_mark, a car beyond the
 * window is held at its edge, so the direction it comes from shows. */
void police_minimap_marks(renderer_t* renderer, const city_map_t* map,
                          const float* cars, size_t count, float left,
                          float top, float side, float u0, float v0, float u1,
                          float v1, float scale, float clock);

/* The same on the full-screen map. */
void police_map_marks(renderer_t* renderer, const city_map_t* map,
                      const float* cars, size_t count, float left, float top,
                      float side, float scale, float clock);

#endif  // GAME_SRC_STAGE_GUIDE_HUD_H_
