/**
 * @file generator_internal.h
 * @brief What the pieces of the city generator share: the layout the parts
 * are placed in, the small types they pass about and the helpers that more
 * than one of them uses. Not for use outside tools/common.
 */
#ifndef TOOLS_COMMON_GENERATOR_INTERNAL_H_
#define TOOLS_COMMON_GENERATOR_INTERNAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "city_generator.h"
#include "city_model.h"
#include "city_package.h"
#include "junction_geometry.h"
#include "mesh_data.h"

typedef struct {
  double x, z;
} vec2_t;

typedef struct {
  city_package_t* package;
  const city_generator_options_t* options;
  double min_x, min_z;
  int columns, rows;
  const city_terrain_t* terrain; /* NULL or cols == 0 for flat ground */
  const city_model_t* model;
  struct arcade_edge* arcade_edges; /* street-facing building edges */
  size_t arcade_edge_count;
} layout_t;

typedef struct {
  double half_width;
  double pavement;
} road_fit_t;

typedef struct {
  double start;
  double end;
} road_cuts_t;

/* The part of a road's centre line that matters when fitting its width
 * around `focus`: only samples within `reach` of it. A reach of zero
 * means the whole road. */
typedef struct {
  local_point_t point;
  double reach;
} focus_t;

#define FIT_REACH \
  12.0 /* metres of road either side of a point that set its width */

typedef struct {
  size_t road;
  bool at_start;
} arm_source_t;

/* A pitched roof over an irregular footprint: the roof is a ridge along
 * the long axis of the footprint's bounding rectangle, falling away on
 * both sides, and the walls rise to meet it. */
typedef struct {
  vec2_t across; /* unit vector across the ridge */
  double mid;    /* across-coordinate of the ridge line */
  double slope;  /* rise per metre away from the ridge */
  double rise;   /* height of the ridge above the eaves */
} ridge_t;

/* Porticoes: how deep they are set back, the pillars' width and spacing. */
#define ARCADE_DEPTH 3.0
#define ARCADE_PILLAR 0.7
#define ARCADE_SPACING 4.5

typedef struct arcade_edge {
  local_point_t a, b;
} arcade_edge_t;

/* The smallest rectangle around a footprint, found by trying every edge
 * direction. `axis` runs along the long side. */
typedef struct {
  vec2_t axis;
  vec2_t across;
  double along_min, along_max, across_min, across_max;
  double area;
} bounding_rectangle_t;

typedef enum { ROOF_FLAT, ROOF_HIPPED, ROOF_GABLED, ROOF_RIDGE } roof_shape_t;

typedef enum { LANDMARK_NONE, LANDMARK_TOWN_HALL, LANDMARK_CHURCH } landmark_t;

/* ---- common ---- */

vec3_t gen_at(local_point_t p, float y);

bool gen_add_vertex(mesh_data_t* mesh, vec3_t position, vec3_t normal, float u,
                    float v);

/* Adds a, b, c wound so the face normal points along `up_hint`. */
bool gen_add_face(mesh_data_t* mesh, uint32_t a, uint32_t b, uint32_t c,
                  vec3_t up_hint);

vec2_t gen_v2(double x, double z);

vec2_t gen_v2_sub(local_point_t a, local_point_t b);

double gen_v2_length(vec2_t v);

vec2_t gen_v2_normalize(vec2_t v);

/* Right-hand side of a direction, seen from above with y up. */
vec2_t gen_v2_right(vec2_t d);

local_point_t gen_offset(local_point_t p, vec2_t d, double amount);

/* Lateral unit offsets per point with mitre at the interior joints. */
void gen_lateral_offsets(const local_point_t* points, size_t count,
                         vec2_t* laterals);

/* A strip between offsets a..b (metres right of the centre line, negative is
 * left) at height y, with uv u across 0..1 and v along in texture repeats. */
bool gen_add_strip(mesh_data_t* mesh, const local_point_t* points,
                   const vec2_t* laterals, size_t count, const double* from,
                   const double* to, float y, float texture_length);

/* A vertical face along the offset line, from y0 to y1, facing `outward`. */
bool gen_add_kerb(mesh_data_t* mesh, const local_point_t* points,
                  const vec2_t* laterals, size_t count, const double* along,
                  float y0, float y1, bool facing_right);

city_chunk_t* gen_chunk_at(layout_t* layout, local_point_t p);

local_point_t gen_centroid(const local_point_t* points, size_t count);

void gen_finish_part(city_chunk_t* chunk, city_part_t* part);

double gen_point_segment_distance(local_point_t p, local_point_t a,
                                  local_point_t b);

bool gen_add_polygon_part(city_chunk_t* chunk, city_part_t* part,
                          const local_point_t* raw, size_t raw_count, float y,
                          float uv_scale);

/* Position `distance` metres along a polyline and the segment it lies on. */
local_point_t gen_along_polyline(const local_point_t* points, size_t count,
                                 double distance, size_t* segment);

bool gen_point_inside(local_point_t p, const local_point_t* ring, size_t count);

bool gen_add_wall_quad(city_part_t* walls, local_point_t p, local_point_t q,
                       float y0, float y1, vec3_t normal, float u0, float u1,
                       float v0, float v1);

bounding_rectangle_t gen_bounding_rectangle(const local_point_t* points,
                                            size_t count);

vec3_t gen_rectangle_point(const bounding_rectangle_t* r, double along,
                           double across, float y);

double gen_terrain_at(const layout_t* layout, double x, double z);

void gen_lift_part(city_part_t* part, float dy);

/* ---- roads ---- */

/* The fit the junctions see: the tighter of the road's two ends. */
road_fit_t gen_fit_road(const city_model_t* model, const city_road_t* road,
                        const city_generator_options_t* o);

/* A flat white quad across a road arm: from `from` to `to` metres out
 * along the arm, between lateral offsets `left` and `right`. */
bool gen_add_marking_quad(mesh_data_t* mesh, local_point_t centre,
                          vec2_t direction, double from, double to, double left,
                          double right, float y);

bool generate_junction(layout_t* layout, const city_model_t* model,
                       const city_junction_t* junction, const road_fit_t* fits,
                       road_cuts_t* cuts, city_generator_stats_t* stats);

bool generate_road(layout_t* layout, const city_model_t* model,
                   const city_road_t* road, road_fit_t fit, road_cuts_t cuts,
                   city_generator_stats_t* stats);

/* ---- buildings ---- */

/* True when edge i of a building faces a named arcaded pedestrian street
 * closely and runs alongside it, on a building deep enough to set a
 * ground floor back. */
bool gen_edge_faces_arcade(const city_model_t* model,
                           const city_generator_options_t* o,
                           const city_building_t* building, size_t i,
                           double depth_available);

double gen_building_depth(const city_building_t* building);

bool generate_building(layout_t* layout, const city_building_t* building,
                       city_generator_stats_t* stats);

/* ---- environment ---- */

/* Zebra stripes at a mapped crossing node on a street cars use, unless a
 * junction is close enough to have its own. */
bool generate_crossings(layout_t* layout, const city_model_t* model,
                        const road_fit_t* fits);

/* Walls, fences and hedges along their mapped lines, left out where they
 * would cross a carriageway. */
bool generate_barriers(layout_t* layout, const city_model_t* model);

bool generate_furniture(layout_t* layout, const city_model_t* model,
                        city_generator_stats_t* stats);

/* The mapped single trees, each a trunk and a crown like a park's, with a
 * collider on the trunk. They sit on the ground where they were mapped. */
bool generate_street_trees(layout_t* layout, const city_model_t* model,
                           city_generator_stats_t* stats);

bool generate_area(layout_t* layout, const city_area_t* area,
                   city_generator_stats_t* stats);

bool generate_ground(layout_t* layout, city_generator_stats_t* stats);

/* Lays everything but the ground and the buildings (already placed) on
 * the terrain: large surfaces are cut into small triangles first so they
 * follow the hills, then every vertex rises by the ground height under
 * it. Chunk bounds are rebuilt. */
bool gen_apply_terrain(layout_t* layout);

#endif  // TOOLS_COMMON_GENERATOR_INTERNAL_H_
