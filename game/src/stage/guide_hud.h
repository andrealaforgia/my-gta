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
  char label[32]; /* shown beside the distance, and on the map */
  vec3_t colour;  /* of its marks and arrow */
} guide_t;

/* A place worth going to, read from a one-line file of the form
 *   latitude longitude LABEL IN CAPITALS
 * (lines starting with # are notes). The place is put on the city's ground
 * using the package's origin. False if the file is not there. */
bool guide_target_load(guide_t* guide, const char* path, double origin_lat,
                       double origin_lon);

/* The compass arrow towards (aim_x, aim_z) and the distance to the place.
 * The player is at (x, z) heading (fx, fz), a unit vector. `row` stacks
 * several under the minimap, from 0. */
void guide_hud_draw(const guide_t* guide, renderer_t* renderer,
                    float screen_width, float screen_height, float x, float z,
                    float fx, float fz, float aim_x, float aim_z, int row);

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
