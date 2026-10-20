#include "city_generator.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "config.h"
#include "junction_geometry.h"
#include "log.h"
#include "physics_world.h"
#include "triangulate.h"

city_generator_options_t city_generator_default_options(void) {
  city_generator_options_t o = {150.0f, 0.06f, true,  2.0f,  0.18f, 8.0f,
                                3.0f,   3.2f,  12,    0.25f, 1.0f,  0.9f,
                                0.7f,   0.6f,  28.0f, 14.0f, {0}};
  return o;
}

bool city_generator_options_load(city_generator_options_t* o,
                                 const char* path) {
  *o = city_generator_default_options();
  config_t config;
  if (!config_load(&config, path)) {
    LOG_ERROR("generate", "cannot read %s", path);
    config_free(&config);
    return false;
  }
  const char* arcades = config_get_string(&config, "arcade_streets", "");
  snprintf(o->arcade_streets, sizeof(o->arcade_streets), "%s", arcades);
  o->chunk_size = config_get_float(&config, "chunk_size", o->chunk_size);
  o->road_height = config_get_float(&config, "road_height", o->road_height);
  o->pavements = config_get_bool(&config, "pavements", o->pavements);
  o->pavement_width =
      config_get_float(&config, "pavement_width", o->pavement_width);
  o->kerb_height = config_get_float(&config, "kerb_height", o->kerb_height);
  o->road_texture_length =
      config_get_float(&config, "road_texture_length", o->road_texture_length);
  o->window_width = config_get_float(&config, "window_width", o->window_width);
  o->storey_height =
      config_get_float(&config, "storey_height", o->storey_height);
  o->facade_variants =
      config_get_int(&config, "facade_variants", o->facade_variants);
  o->ground_uv_scale =
      config_get_float(&config, "ground_uv_scale", o->ground_uv_scale);
  o->road_friction =
      config_get_float(&config, "road_friction", o->road_friction);
  o->pavement_friction =
      config_get_float(&config, "pavement_friction", o->pavement_friction);
  o->ground_friction =
      config_get_float(&config, "ground_friction", o->ground_friction);
  o->building_friction =
      config_get_float(&config, "building_friction", o->building_friction);
  o->lamp_spacing = config_get_float(&config, "lamp_spacing", o->lamp_spacing);
  o->tree_spacing = config_get_float(&config, "tree_spacing", o->tree_spacing);
  if (config_defaulted_count(&config) > 0)
    LOG_WARN("generate", "%s: %zu settings missing, defaults used", path,
             config_defaulted_count(&config));
  config_free(&config);
  bool valid = o->chunk_size >= 20 && o->facade_variants >= 1;
  if (!valid)
    LOG_ERROR("generate",
              "%s: chunk_size must be >= 20 and facade_variants >= 1", path);
  return valid;
}

uint64_t city_generator_options_hash(const city_generator_options_t* o) {
  /* FNV-1a over a textual rendering so padding bytes do not matter. */
  char text[512];
  snprintf(text, sizeof(text),
           "%g %g %d %g %g %g %g %g %d %g %g %g %g %g %g %g %s", o->chunk_size,
           o->road_height, o->pavements, o->pavement_width, o->kerb_height,
           o->road_texture_length, o->window_width, o->storey_height,
           o->facade_variants, o->ground_uv_scale, o->road_friction,
           o->pavement_friction, o->ground_friction, o->building_friction,
           o->lamp_spacing, o->tree_spacing, o->arcade_streets);
  uint64_t hash = 1469598103934665603ull;
  for (const char* c = text; *c; c++)
    hash = (hash ^ (uint8_t)*c) * 1099511628211ull;
  return hash;
}

/* ---- mesh helpers ---- */

static vec3_t at(local_point_t p, float y) {
  return vec3((float)p.x, y, (float)p.z);
}

static bool add_vertex(mesh_data_t* mesh, vec3_t position, vec3_t normal,
                       float u, float v) {
  vertex_t vertex = {position, normal, u, v};
  return mesh_data_add_vertex(mesh, vertex);
}

/* Adds a, b, c wound so the face normal points along `up_hint`. */
static bool add_face(mesh_data_t* mesh, uint32_t a, uint32_t b, uint32_t c,
                     vec3_t up_hint) {
  vec3_t pa = mesh->vertices[a].position, pb = mesh->vertices[b].position,
         pc = mesh->vertices[c].position;
  vec3_t normal = vec3_cross(vec3_sub(pb, pa), vec3_sub(pc, pa));
  if (vec3_dot(normal, up_hint) < 0)
    return mesh_data_add_triangle(mesh, a, c, b);
  return mesh_data_add_triangle(mesh, a, b, c);
}

/* ---- geometry in the plane: x east, z south ---- */

typedef struct {
  double x, z;
} vec2_t;

static vec2_t v2(double x, double z) {
  vec2_t v = {x, z};
  return v;
}
static vec2_t v2_sub(local_point_t a, local_point_t b) {
  return v2(a.x - b.x, a.z - b.z);
}
static double v2_length(vec2_t v) { return hypot(v.x, v.z); }
static vec2_t v2_normalize(vec2_t v) {
  double length = v2_length(v);
  return length > 1e-9 ? v2(v.x / length, v.z / length) : v2(0, 0);
}
/* Right-hand side of a direction, seen from above with y up. */
static vec2_t v2_right(vec2_t d) { return v2(-d.z, d.x); }
static local_point_t offset(local_point_t p, vec2_t d, double amount) {
  local_point_t q = {p.x + d.x * amount, p.z + d.z * amount};
  return q;
}

/* Lateral unit offsets per point with mitre at the interior joints. */
static void lateral_offsets(const local_point_t* points, size_t count,
                            vec2_t* laterals) {
  for (size_t i = 0; i < count; i++) {
    vec2_t before =
        i > 0 ? v2_normalize(v2_sub(points[i], points[i - 1])) : v2(0, 0);
    vec2_t after = i + 1 < count
                       ? v2_normalize(v2_sub(points[i + 1], points[i]))
                       : v2(0, 0);
    vec2_t direction = v2_normalize(v2(before.x + after.x, before.z + after.z));
    if (v2_length(direction) < 1e-6) direction = i > 0 ? before : after;
    vec2_t right = v2_right(direction);
    if (i > 0 && i + 1 < count) {
      double cosine = before.x * after.x + before.z * after.z;
      double half = sqrt(fmax((1.0 + cosine) * 0.5, 0.05)); /* cos(theta / 2) */
      double scale = fmin(1.0 / half, 2.5);
      right = v2(right.x * scale, right.z * scale);
    }
    laterals[i] = right;
  }
}

/* A strip between offsets a..b (metres right of the centre line, negative is
 * left) at height y, with uv u across 0..1 and v along in texture repeats. */
static bool add_strip(mesh_data_t* mesh, const local_point_t* points,
                      const vec2_t* laterals, size_t count, const double* from,
                      const double* to, float y, float texture_length) {
  uint32_t base = (uint32_t)mesh->vertex_count;
  double distance = 0;
  for (size_t i = 0; i < count; i++) {
    if (i > 0) distance += v2_length(v2_sub(points[i], points[i - 1]));
    float v = (float)(distance / texture_length);
    if (!add_vertex(mesh, at(offset(points[i], laterals[i], from[i]), y),
                    vec3(0, 1, 0), 0, v))
      return false;
    if (!add_vertex(mesh, at(offset(points[i], laterals[i], to[i]), y),
                    vec3(0, 1, 0), 1, v))
      return false;
  }
  for (size_t i = 0; i + 1 < count; i++) {
    uint32_t a = base + (uint32_t)i * 2, b = a + 1, c = a + 2, d = a + 3;
    if (!add_face(mesh, a, b, d, vec3(0, 1, 0)) ||
        !add_face(mesh, a, d, c, vec3(0, 1, 0)))
      return false;
  }
  return true;
}

/* A vertical face along the offset line, from y0 to y1, facing `outward`. */
static bool add_kerb(mesh_data_t* mesh, const local_point_t* points,
                     const vec2_t* laterals, size_t count, const double* along,
                     float y0, float y1, bool facing_right) {
  uint32_t base = (uint32_t)mesh->vertex_count;
  for (size_t i = 0; i < count; i++) {
    vec2_t right = v2_normalize(laterals[i]);
    vec3_t normal = vec3((float)(facing_right ? right.x : -right.x), 0,
                         (float)(facing_right ? right.z : -right.z));
    local_point_t p = offset(points[i], laterals[i], along[i]);
    if (!add_vertex(mesh, at(p, y0), normal, (float)i, 0) ||
        !add_vertex(mesh, at(p, y1), normal, (float)i, 1))
      return false;
  }
  for (size_t i = 0; i + 1 < count; i++) {
    uint32_t a = base + (uint32_t)i * 2, b = a + 1, c = a + 2, d = a + 3;
    vec3_t hint = mesh->vertices[a].normal;
    if (!add_face(mesh, a, b, d, hint) || !add_face(mesh, a, d, c, hint))
      return false;
  }
  return true;
}

/* ---- chunk placement ---- */

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

static city_chunk_t* chunk_at(layout_t* layout, local_point_t p) {
  int column = (int)floor((p.x - layout->min_x) / layout->options->chunk_size);
  int row = (int)floor((p.z - layout->min_z) / layout->options->chunk_size);
  if (column < 0) column = 0;
  if (row < 0) row = 0;
  if (column >= layout->columns) column = layout->columns - 1;
  if (row >= layout->rows) row = layout->rows - 1;
  return city_package_chunk(layout->package, column, row);
}

static local_point_t centroid(const local_point_t* points, size_t count) {
  local_point_t c = {0, 0};
  for (size_t i = 0; i < count; i++) {
    c.x += points[i].x;
    c.z += points[i].z;
  }
  c.x /= (double)count;
  c.z /= (double)count;
  return c;
}

static void finish_part(city_chunk_t* chunk, city_part_t* part) {
  chunk->bounds = aabb_merge(chunk->bounds, mesh_data_bounds(&part->mesh));
}

/* ---- features ---- */

/* ---- widths fitted to the buildings ---- */

typedef struct {
  double half_width;
  double pavement;
} road_fit_t;

typedef struct {
  double start;
  double end;
} road_cuts_t;

static double point_segment_distance(local_point_t p, local_point_t a,
                                     local_point_t b) {
  double abx = b.x - a.x, abz = b.z - a.z;
  double length_squared = abx * abx + abz * abz;
  double t = length_squared > 1e-12
                 ? ((p.x - a.x) * abx + (p.z - a.z) * abz) / length_squared
                 : 0;
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  return hypot(p.x - (a.x + abx * t), p.z - (a.z + abz * t));
}

/* The part of a road's centre line that matters when fitting its width
 * around `focus`: only samples within `reach` of it. A reach of zero
 * means the whole road. */
typedef struct {
  local_point_t point;
  double reach;
} focus_t;

static bool in_focus(focus_t focus, local_point_t sample) {
  return focus.reach <= 0 || hypot(sample.x - focus.point.x,
                                   sample.z - focus.point.z) <= focus.reach;
}

/* The nearest building edge to the road's centre line, sampled every two
 * metres, or a large number when nothing is near. */
static double corridor_half_width(const city_model_t* model,
                                  const city_road_t* road, double reach,
                                  focus_t focus) {
  double min_x = 1e30, max_x = -1e30, min_z = 1e30, max_z = -1e30;
  for (size_t i = 0; i < road->point_count; i++) {
    min_x = fmin(min_x, road->points[i].x),
    max_x = fmax(max_x, road->points[i].x);
    min_z = fmin(min_z, road->points[i].z),
    max_z = fmax(max_z, road->points[i].z);
  }
  if (focus.reach > 0) {
    min_x = fmax(min_x, focus.point.x - focus.reach);
    max_x = fmin(max_x, focus.point.x + focus.reach);
    min_z = fmax(min_z, focus.point.z - focus.reach);
    max_z = fmin(max_z, focus.point.z + focus.reach);
  }
  double nearest = 1e30;
  for (size_t b = 0; b < model->building_count; b++) {
    const city_building_t* building = &model->buildings[b];
    bool near_road = false;
    for (size_t i = 0; i < building->point_count && !near_road; i++) {
      local_point_t p = building->points[i];
      near_road = p.x > min_x - reach && p.x < max_x + reach &&
                  p.z > min_z - reach && p.z < max_z + reach;
    }
    if (!near_road) continue;
    for (size_t i = 0; i < building->point_count; i++) {
      local_point_t a = building->points[i];
      local_point_t c = building->points[(i + 1) % building->point_count];
      for (size_t r = 1; r < road->point_count; r++) {
        local_point_t p = road->points[r - 1], q = road->points[r];
        double length = hypot(q.x - p.x, q.z - p.z);
        int samples = (int)(length / 2.0) + 1;
        for (int k = 0; k <= samples; k++) {
          double t = (double)k / (double)samples;
          local_point_t sample = {p.x + (q.x - p.x) * t, p.z + (q.z - p.z) * t};
          if (!in_focus(focus, sample)) continue;
          double d = point_segment_distance(sample, a, c);
          if (d < nearest) nearest = d;
        }
      }
    }
  }
  return nearest;
}

/* Half the gap to the nearest other road's centre line that shares no
 * node with this one (paths and streams excepted), or a large number. */
static double corridor_to_roads(const city_model_t* model,
                                const city_road_t* road, double reach,
                                focus_t focus) {
  double nearest = 1e30;
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* other = &model->roads[r];
    if (other == road || other->class == ROAD_CLASS_PATH ||
        other->class == ROAD_CLASS_WATERWAY)
      continue;
    if (focus.reach > 0) {
      bool near = false;
      for (size_t k = 1; k < other->point_count && !near; k++)
        near = point_segment_distance(focus.point, other->points[k - 1],
                                      other->points[k]) < focus.reach + reach;
      if (!near) continue;
    }
    bool shares_node = false;
    for (size_t i = 0; i < road->point_count && !shares_node; i++)
      for (size_t k = 0; k < other->point_count && !shares_node; k++)
        if (road->node_ids[i] != 0 && road->node_ids[i] == other->node_ids[k])
          shares_node = true;
    if (shares_node) continue;
    for (size_t i = 1; i < road->point_count; i++) {
      local_point_t p = road->points[i - 1], q = road->points[i];
      vec2_t direction = v2_normalize(v2_sub(q, p));
      double length = hypot(q.x - p.x, q.z - p.z);
      int samples = (int)(length / 3.0) + 1;
      for (int k = 0; k <= samples; k++) {
        double t = (double)k / (double)samples;
        local_point_t sample = {p.x + (q.x - p.x) * t, p.z + (q.z - p.z) * t};
        if (!in_focus(focus, sample)) continue;
        for (size_t m = 1; m < other->point_count; m++) {
          local_point_t a = other->points[m - 1], b = other->points[m];
          vec2_t other_direction = v2_normalize(v2_sub(b, a));
          /* Only a road running alongside counts: nearly parallel, and
           * the sample lies beside the segment rather than past its end. */
          double cosine =
              direction.x * other_direction.x + direction.z * other_direction.z;
          if (fabs(cosine) < 0.9) continue;
          double abx = b.x - a.x, abz = b.z - a.z;
          double length2 = abx * abx + abz * abz;
          if (length2 < 1e-9) continue;
          double u =
              ((sample.x - a.x) * abx + (sample.z - a.z) * abz) / length2;
          if (u < 0 || u > 1) continue;
          double d = point_segment_distance(sample, a, b);
          if (d < reach && d < nearest) nearest = d;
        }
      }
    }
  }
  return nearest * 0.5;
}

static road_fit_t fit_at(const city_model_t* model, const city_road_t* road,
                         const city_generator_options_t* o, focus_t focus) {
  road_fit_t fit;
  fit.half_width = road->width * 0.5;
  bool has_pavement = o->pavements && road->class != ROAD_CLASS_SERVICE &&
                      road->class != ROAD_CLASS_PEDESTRIAN &&
                      road->class != ROAD_CLASS_PATH &&
                      road->class != ROAD_CLASS_WATERWAY;
  fit.pavement = has_pavement ? o->pavement_width : 0;
  if (road->class == ROAD_CLASS_PATH || road->class == ROAD_CLASS_WATERWAY)
    return fit;
  const double gap = 0.3, narrowest = 1.75;
  /* Another road close alongside: keep this road's width and give up
   * the pavement on the gap, so two carriageways meet at a median. */
  double to_roads = corridor_to_roads(
      model, road, 2.0 * (fit.half_width + fit.pavement + 1.0), focus);
  if (to_roads < fit.half_width + fit.pavement + gap) {
    double available = fmax(to_roads - 0.1, narrowest);
    fit.half_width = fmin(fit.half_width, available);
    fit.pavement = fmin(fmax(available - fit.half_width, 0.0), fit.pavement);
    if (fit.pavement < 0.4) fit.pavement = 0;
  }
  double wanted = fit.half_width + fit.pavement + gap;
  double corridor = corridor_half_width(model, road, wanted + 1.0, focus);
  if (corridor >= wanted) return fit;
  double available = fmax(corridor - gap, narrowest);
  fit.half_width =
      fmin(fit.half_width, fmax(narrowest, available - fit.pavement));
  fit.pavement = fmin(fmax(available - fit.half_width, 0.0), fit.pavement);
  if (fit.pavement < 0.4) fit.pavement = 0;
  return fit;
}

#define FIT_REACH \
  12.0 /* metres of road either side of a point that set its width */

/* The fit the junctions see: the tighter of the road's two ends. */
static road_fit_t fit_road(const city_model_t* model, const city_road_t* road,
                           const city_generator_options_t* o) {
  focus_t start = {road->points[0], FIT_REACH};
  focus_t end = {road->points[road->point_count - 1], FIT_REACH};
  road_fit_t a = fit_at(model, road, o, start), b = fit_at(model, road, o, end);
  road_fit_t fit = {fmin(a.half_width, b.half_width),
                    fmin(a.pavement, b.pavement)};
  return fit;
}

/* Widths along a road: each point fitted to what stands near it, never
 * wider than the fit at the ends where the junction geometry was laid
 * out, and never changing faster than a gentle taper. A road with no
 * pavement at its ends has none anywhere; one with pavements keeps at
 * least a sliver so the strips never collapse. */
static void fit_along(const city_model_t* model, const city_road_t* road,
                      const city_generator_options_t* o, road_fit_t ends,
                      const local_point_t* points, size_t count, double* halves,
                      double* pavements) {
  for (size_t i = 0; i < count; i++) {
    focus_t focus = {points[i], FIT_REACH};
    road_fit_t fit = fit_at(model, road, o, focus);
    halves[i] = fit.half_width;
    pavements[i] = ends.pavement > 0 ? fmax(fit.pavement, 0.3) : 0;
  }
  halves[0] = ends.half_width, halves[count - 1] = ends.half_width;
  pavements[0] = ends.pavement, pavements[count - 1] = ends.pavement;
  const double taper = 0.15; /* metres of width per metre of road */
  for (int pass = 0; pass < 2; pass++) {
    for (size_t i = 1; i < count; i++) {
      size_t k = pass == 0 ? i : count - 1 - i,
             prev = pass == 0 ? k - 1 : k + 1;
      double d =
          hypot(points[k].x - points[prev].x, points[k].z - points[prev].z);
      halves[k] = fmin(halves[k], halves[prev] + taper * d);
      double outer = fmin(halves[k] + pavements[k],
                          halves[prev] + pavements[prev] + taper * d);
      if (pavements[k] > 0) pavements[k] = fmax(outer - halves[k], 0.3);
    }
  }
}

/* ---- junctions ---- */

/* Drops repeated consecutive points, and the last if it repeats the first. */
static size_t dedupe_ring(const local_point_t* in, size_t count,
                          local_point_t* out) {
  size_t n = 0;
  for (size_t i = 0; i < count; i++) {
    if (n > 0 && hypot(in[i].x - out[n - 1].x, in[i].z - out[n - 1].z) < 1e-4)
      continue;
    out[n++] = in[i];
  }
  while (n > 1 &&
         hypot(out[n - 1].x - out[0].x, out[n - 1].z - out[0].z) < 1e-4)
    n--;
  return n;
}

static bool add_polygon_part(city_chunk_t* chunk, city_part_t* part,
                             const local_point_t* raw, size_t raw_count,
                             float y, float uv_scale) {
  local_point_t polygon[JUNCTION_MAX_POLYGON];
  size_t count = dedupe_ring(raw, raw_count, polygon);
  if (count < 3 || fabs(polygon_signed_area(polygon, count)) < 0.01)
    return true;
  uint32_t* indices = malloc(3 * (count - 2) * sizeof(uint32_t));
  if (!indices) return false;
  bool clean;
  size_t triangles = triangulate_polygon(polygon, count, indices, &clean);
  uint32_t base = (uint32_t)part->mesh.vertex_count;
  for (size_t i = 0; i < count; i++)
    if (!add_vertex(&part->mesh, at(polygon[i], y), vec3(0, 1, 0),
                    (float)(polygon[i].x * uv_scale),
                    (float)(polygon[i].z * uv_scale))) {
      free(indices);
      return false;
    }
  for (size_t t = 0; t < triangles; t++) {
    local_point_t tri[3] = {polygon[indices[t * 3]],
                            polygon[indices[t * 3 + 1]],
                            polygon[indices[t * 3 + 2]]};
    if (fabs(polygon_signed_area(tri, 3)) < 1e-4) continue; /* a sliver */
    if (!add_face(&part->mesh, base + indices[t * 3], base + indices[t * 3 + 1],
                  base + indices[t * 3 + 2], vec3(0, 1, 0))) {
      free(indices);
      return false;
    }
  }
  free(indices);
  finish_part(chunk, part);
  return true;
}

/* A kerb face along a polyline of points on the road edge, facing the
 * road (which is on the `road_side` of the line's direction). */
static bool add_kerb_line(mesh_data_t* mesh, const local_point_t* points,
                          size_t count, float y0, float y1,
                          local_point_t road_point) {
  for (size_t i = 1; i < count; i++) {
    local_point_t a = points[i - 1], b = points[i];
    if (hypot(b.x - a.x, b.z - a.z) < 1e-4) continue;
    vec2_t d = v2_normalize(v2_sub(b, a));
    vec2_t r = v2_right(d);
    /* Face the kerb towards the road. */
    vec2_t to_road = v2_normalize(v2_sub(road_point, a));
    bool road_on_right = to_road.x * r.x + to_road.z * r.z > 0;
    vec3_t normal = vec3((float)(road_on_right ? r.x : -r.x), 0,
                         (float)(road_on_right ? r.z : -r.z));
    uint32_t base = (uint32_t)mesh->vertex_count;
    if (!add_vertex(mesh, at(a, y0), normal, 0, 0) ||
        !add_vertex(mesh, at(b, y0), normal, 1, 0) ||
        !add_vertex(mesh, at(b, y1), normal, 1, 1) ||
        !add_vertex(mesh, at(a, y1), normal, 0, 1))
      return false;
    if (!add_face(mesh, base, base + 1, base + 2, normal) ||
        !add_face(mesh, base, base + 2, base + 3, normal))
      return false;
  }
  return true;
}

typedef struct {
  size_t road;
  bool at_start;
} arm_source_t;

/* A flat white quad across a road arm: from `from` to `to` metres out
 * along the arm, between lateral offsets `left` and `right`. */
static bool add_marking_quad(mesh_data_t* mesh, local_point_t centre,
                             vec2_t direction, double from, double to,
                             double left, double right, float y) {
  vec2_t side = v2_right(direction);
  vec3_t a = at(offset(offset(centre, direction, from), side, left), y);
  vec3_t b = at(offset(offset(centre, direction, from), side, right), y);
  vec3_t c = at(offset(offset(centre, direction, to), side, right), y);
  vec3_t d = at(offset(offset(centre, direction, to), side, left), y);
  /* An arm with no direction, or a line narrower than it is long, would
   * make slivers: skip rather than paint a degenerate quad. */
  if (vec3_length(vec3_cross(vec3_sub(b, a), vec3_sub(d, a))) < 1e-4f)
    return true;
  uint32_t base = (uint32_t)mesh->vertex_count;
  vec3_t up = vec3(0, 1, 0);
  if (!add_vertex(mesh, a, up, 0, 0) || !add_vertex(mesh, b, up, 1, 0) ||
      !add_vertex(mesh, c, up, 1, 1) || !add_vertex(mesh, d, up, 0, 1))
    return false;
  return add_face(mesh, base, base + 1, base + 2, up) &&
         add_face(mesh, base, base + 2, base + 3, up);
}

/* Stop lines and zebra crossings on every drivable arm of a junction of
 * three or more roads, just past the arm's cut: a line across the road,
 * then stripes where the arm has pavements for people to cross between. */
static int road_rank(road_class_t class) {
  switch (class) {
    case ROAD_CLASS_MOTORWAY:
      return 0;
    case ROAD_CLASS_PRIMARY:
      return 1;
    case ROAD_CLASS_SECONDARY:
      return 2;
    case ROAD_CLASS_TERTIARY:
      return 3;
    default:
      return 4;
  }
}

/* A stop sign at the end of each side street that meets a bigger road: a
 * pole on the pavement just short of the junction, on the right as the
 * driver sees it, carrying a red plate that faces the oncoming car. */
static bool generate_stop_signs(layout_t* layout, const city_model_t* model,
                                const city_junction_t* junction,
                                const junction_arm_t* arms,
                                const arm_source_t* sources, size_t arm_count,
                                const junction_layout_t* shape,
                                const road_fit_t* fits) {
  const city_generator_options_t* o = layout->options;
  int best = 99;
  for (size_t a = 0; a < arm_count; a++) {
    road_class_t c = model->roads[sources[a].road].class;
    if (c == ROAD_CLASS_PEDESTRIAN) continue;
    best = best < road_rank(c) ? best : road_rank(c);
  }
  if (arm_count < 3) return true;
  city_chunk_t* chunk = chunk_at(layout, junction->position);
  city_part_t* poles = NULL;
  city_part_t* plates = NULL;
  for (size_t a = 0; a < arm_count; a++) {
    const city_road_t* road = &model->roads[sources[a].road];
    if (road->class == ROAD_CLASS_PEDESTRIAN || road_rank(road->class) <= best)
      continue;
    if (polyline_length(road->points, road->point_count) < 25.0 ||
        fits[sources[a].road].pavement <= 0)
      continue;
    if (!poles) {
      poles = city_chunk_add_part(chunk, "sign_pole", PART_COLLIDER_MESH,
                                  COLLISION_LAYER_PROP, 0.5f,
                                  (uint64_t)junction->node_id);
      plates = poles
                   ? city_chunk_add_part(chunk, "sign_red", PART_COLLIDER_NONE,
                                         0, 0, (uint64_t)junction->node_id)
                   : NULL;
      if (!poles || !plates) return false;
      poles = &chunk->parts[chunk->part_count - 2];
    }
    vec2_t d = {arms[a].direction.x, arms[a].direction.z};
    /* A car heading into the junction travels along -d; its right is
     * (dz, -dx) in this frame (x east, z south). */
    vec2_t right = v2(d.z, -d.x);
    double half = fits[sources[a].road].half_width;
    local_point_t foot = offset(
        offset(junction->position, d, shape->cut[a] + 1.4), right, half + 0.7);
    float y0 = o->kerb_height;
    if (!mesh_data_add_box(&poles->mesh,
                           vec3((float)foot.x, y0 + 1.1f, (float)foot.z),
                           vec3(0.035f, 1.1f, 0.035f)))
      return false;
    /* The plate: thin across the heading, square on to the car. */
    vec3_t half_extents =
        vec3(0.3f * (float)fabs(right.x) + 0.015f * (float)fabs(d.x), 0.3f,
             0.3f * (float)fabs(right.z) + 0.015f * (float)fabs(d.z));
    if (!mesh_data_add_box(&plates->mesh,
                           vec3((float)foot.x, y0 + 2.0f, (float)foot.z),
                           half_extents))
      return false;
  }
  if (!poles) return true;
  finish_part(chunk, poles);
  finish_part(chunk, plates);
  return true;
}

static bool generate_markings(layout_t* layout, const city_model_t* model,
                              const city_junction_t* junction,
                              const junction_arm_t* arms,
                              const arm_source_t* sources, size_t arm_count,
                              const junction_layout_t* shape,
                              const road_fit_t* fits) {
  const city_generator_options_t* o = layout->options;
  if (arm_count < 3) return true;
  city_chunk_t* chunk = chunk_at(layout, junction->position);
  city_part_t* part = city_chunk_add_part(chunk, "marking", PART_COLLIDER_NONE,
                                          0, 0, (uint64_t)junction->node_id);
  if (!part) return false;
  float y = o->road_height + 0.004f;
  for (size_t a = 0; a < arm_count; a++) {
    const city_road_t* road = &model->roads[sources[a].road];
    if (road->class == ROAD_CLASS_PEDESTRIAN) continue;
    /* Short links between two close junctions get no paint at all, so
     * crossings are not stacked a few metres apart. */
    double road_length = polyline_length(road->points, road->point_count);
    if (road_length < 12.0) continue;
    double half = fits[sources[a].road].half_width;
    vec2_t direction = {arms[a].direction.x, arms[a].direction.z};
    double start = shape->cut[a] + 0.3;
    /* The stop line, on the entering side only: the right half for a
     * two-way road, the whole width for a one-way one. */
    if (!add_marking_quad(&part->mesh, junction->position, direction, start,
                          start + 0.35, road->oneway ? -half + 0.3 : 0.15,
                          half - 0.3, y))
      return false;
    if (fits[sources[a].road].pavement <= 0 || road_length < 25.0) continue;
    double zebra_from = start + 0.9;
    for (double across = -half + 0.4; across + 0.5 <= half - 0.4; across += 1.0)
      if (!add_marking_quad(&part->mesh, junction->position, direction,
                            zebra_from, zebra_from + 2.4, across, across + 0.5,
                            y))
        return false;
  }
  if (part->mesh.vertex_count == 0) {
    mesh_data_free(&part->mesh);
    chunk->part_count--;
  } else {
    finish_part(chunk, part);
  }
  return generate_stop_signs(layout, model, junction, arms, sources, arm_count,
                             shape, fits);
}

static bool generate_junction(layout_t* layout, const city_model_t* model,
                              const city_junction_t* junction,
                              const road_fit_t* fits, road_cuts_t* cuts,
                              city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  junction_arm_t arms[JUNCTION_MAX_ARMS];
  arm_source_t sources[JUNCTION_MAX_ARMS];
  size_t arm_count = 0;
  for (size_t r = 0; r < model->road_count && arm_count < JUNCTION_MAX_ARMS;
       r++) {
    const city_road_t* road = &model->roads[r];
    for (int end = 0; end < 2 && arm_count < JUNCTION_MAX_ARMS; end++) {
      size_t index = end ? road->point_count - 1 : 0;
      if (road->node_ids[index] != junction->node_id) continue;
      if (road->class == ROAD_CLASS_PATH || road->class == ROAD_CLASS_WATERWAY)
        continue; /* paths and streams lie on top or below */
      size_t neighbour = end ? road->point_count - 2 : 1;
      vec2_t direction =
          v2_normalize(v2_sub(road->points[neighbour], road->points[index]));
      arms[arm_count].direction.x = direction.x;
      arms[arm_count].direction.z = direction.z;
      arms[arm_count].half_width = fits[r].half_width;
      arms[arm_count].pavement = fits[r].pavement;
      sources[arm_count].road = r;
      sources[arm_count].at_start = end == 0;
      arm_count++;
    }
  }
  if (arm_count < 2) return true;
  junction_layout_t shape;
  bool patch =
      junction_layout(junction->position, arms, arm_count, 0.2, &shape);
  for (size_t a = 0; a < arm_count; a++) {
    road_cuts_t* cut = &cuts[sources[a].road];
    if (sources[a].at_start)
      cut->start = shape.cut[a];
    else
      cut->end = shape.cut[a];
  }
  if (!patch) return true;

  city_chunk_t* chunk = chunk_at(layout, junction->position);
  bool all_pedestrian = true;
  for (size_t a = 0; a < arm_count; a++)
    if (model->roads[sources[a].road].class != ROAD_CLASS_PEDESTRIAN)
      all_pedestrian = false;
  city_part_t* part = city_chunk_add_part(
      chunk, all_pedestrian ? "paving" : "junction", PART_COLLIDER_MESH,
      COLLISION_LAYER_GROUND, o->road_friction, (uint64_t)junction->node_id);
  if (!part) return false;
  if (!add_polygon_part(chunk, part, shape.polygon, shape.polygon_count,
                        o->road_height, 0.1f))
    return false;
  stats->junction_patches++;
  if (shape.corner_count == 0)
    return generate_markings(layout, model, junction, arms, sources, arm_count,
                             &shape, fits);
  city_part_t* pavement = city_chunk_add_part(
      chunk, "pavement", PART_COLLIDER_MESH, COLLISION_LAYER_GROUND,
      o->pavement_friction, (uint64_t)junction->node_id);
  city_part_t* kerb =
      pavement ? city_chunk_add_part(chunk, "kerb", PART_COLLIDER_NONE, 0, 0,
                                     (uint64_t)junction->node_id)
               : NULL;
  if (!pavement || !kerb) return false;
  pavement = &chunk->parts[chunk->part_count - 2];
  for (size_t c = 0; c < shape.corner_count; c++) {
    const pavement_corner_t* corner = &shape.corners[c];
    if (!add_polygon_part(chunk, pavement, corner->polygon, corner->count,
                          o->kerb_height, 0.5f))
      return false;
    if (!add_kerb_line(&kerb->mesh, corner->kerb, corner->kerb_count,
                       o->road_height, o->kerb_height, junction->position))
      return false;
  }
  finish_part(chunk, kerb);
  return generate_markings(layout, model, junction, arms, sources, arm_count,
                           &shape, fits);
}

/* ---- street furniture ---- */

/* Position `distance` metres along a polyline and the segment it lies on. */
static local_point_t along_polyline(const local_point_t* points, size_t count,
                                    double distance, size_t* segment) {
  double walked = 0;
  for (size_t i = 1; i < count; i++) {
    double step =
        hypot(points[i].x - points[i - 1].x, points[i].z - points[i - 1].z);
    if (walked + step >= distance || i == count - 1) {
      double t = step > 1e-12 ? (distance - walked) / step : 0;
      if (t < 0) t = 0;
      if (t > 1) t = 1;
      local_point_t p = {points[i - 1].x + (points[i].x - points[i - 1].x) * t,
                         points[i - 1].z + (points[i].z - points[i - 1].z) * t};
      *segment = i;
      return p;
    }
    walked += step;
  }
  *segment = count - 1;
  return points[count - 1];
}

/* Lamp posts on the pavement, alternating sides, every lamp_spacing
 * metres along the road: a post, an arm over the road and a head, with
 * a collider so the car can hit them. */
static bool generate_lamps(layout_t* layout, const city_road_t* road,
                           const local_point_t* points, const vec2_t* laterals,
                           size_t count, const double* halves,
                           const double* pavements,
                           city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  double length = polyline_length(points, count);
  if (length < o->lamp_spacing * 0.6) return true;
  city_chunk_t* chunk = chunk_at(layout, centroid(points, count));
  city_part_t* part = city_chunk_add_part(chunk, "lamp", PART_COLLIDER_MESH,
                                          COLLISION_LAYER_PROP, 0.5f,
                                          (uint64_t)road->source_id);
  city_part_t* lights =
      part ? city_chunk_add_part(chunk, "lamp_light", PART_COLLIDER_NONE, 0, 0,
                                 (uint64_t)road->source_id)
           : NULL;
  if (!part || !lights) return false;
  part = &chunk->parts[chunk->part_count - 2];
  const float height = 4.5f;
  int side = 1;
  for (double along = o->lamp_spacing * 0.5; along < length;
       along += o->lamp_spacing, side = -side) {
    size_t segment;
    local_point_t on_line = along_polyline(points, count, along, &segment);
    vec2_t lateral = v2_normalize(laterals[segment > 0 ? segment - 1 : 0]);
    size_t at_point = segment < count ? segment : count - 1;
    double post_offset = side * (halves[at_point] + pavements[at_point] * 0.5);
    local_point_t base = offset(on_line, lateral, post_offset);
    local_point_t head = offset(on_line, lateral, post_offset - side * 0.9);
    float y0 = o->kerb_height;
    if (!mesh_data_add_box(
            &part->mesh, vec3((float)base.x, y0 + height * 0.5f, (float)base.z),
            vec3(0.07f, height * 0.5f, 0.07f)))
      return false;
    vec3_t arm_centre = vec3((float)((base.x + head.x) * 0.5), y0 + height,
                             (float)((base.z + head.z) * 0.5));
    vec3_t arm_half = vec3((float)fabs(head.x - base.x) * 0.5f + 0.06f, 0.06f,
                           (float)fabs(head.z - base.z) * 0.5f + 0.06f);
    if (!mesh_data_add_box(&part->mesh, arm_centre, arm_half)) return false;
    if (!mesh_data_add_box(
            &lights->mesh,
            vec3((float)head.x, y0 + height - 0.12f, (float)head.z),
            vec3(0.22f, 0.1f, 0.22f)))
      return false;
    stats->lamps++;
  }
  if (part->mesh.vertex_count == 0) {
    mesh_data_free(&part->mesh);
    mesh_data_free(&lights->mesh);
    chunk->part_count -= 2;
    return true;
  }
  finish_part(chunk, part);
  finish_part(chunk, lights);
  return true;
}

/* ---- roads ---- */

/* Points every `step` metres along straight stretches, so the width fit
 * sees what stands beside the middle of a long segment, not just its
 * ends. */
static size_t densify(const local_point_t* in, size_t count, double step,
                      local_point_t* out) {
  size_t n = 0;
  for (size_t i = 0; i < count; i++) {
    if (i > 0) {
      local_point_t a = in[i - 1], b = in[i];
      int pieces = (int)(hypot(b.x - a.x, b.z - a.z) / step);
      for (int k = 1; k <= pieces; k++) {
        double t = (double)k / (double)(pieces + 1);
        local_point_t q = {a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t};
        out[n++] = q;
      }
    }
    out[n++] = in[i];
  }
  return n;
}

/* True where a point lies on the carriageway or pavement of a road that
 * cars drive on, other than `self`. Footpaths are not drawn there. */
static bool on_a_street(const city_model_t* model, const city_road_t* self,
                        local_point_t p, double pavement_width) {
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* other = &model->roads[r];
    if (other == self || other->class == ROAD_CLASS_PATH ||
        other->class == ROAD_CLASS_WATERWAY ||
        other->class == ROAD_CLASS_PEDESTRIAN)
      continue;
    double reach = other->width * 0.5 + pavement_width;
    for (size_t k = 1; k < other->point_count; k++)
      if (point_segment_distance(p, other->points[k - 1], other->points[k]) <
          reach)
        return true;
  }
  return false;
}

/* Round iron covers in the carriageway every forty metres or so, a little
 * off the middle of a lane. */
static bool generate_manholes(layout_t* layout, const city_road_t* road,
                              const local_point_t* points,
                              const vec2_t* laterals, size_t count,
                              const double* halves) {
  const city_generator_options_t* o = layout->options;
  double length = polyline_length(points, count);
  if (length < 20.0 || road->class == ROAD_CLASS_MOTORWAY) return true;
  city_chunk_t* chunk = chunk_at(layout, centroid(points, count));
  city_part_t* part = city_chunk_add_part(chunk, "manhole", PART_COLLIDER_NONE,
                                          0, 0, (uint64_t)road->source_id);
  if (!part) return false;
  uint64_t seed = (uint64_t)road->source_id * 0x2545f4914f6cdd1dull + 5;
  float y = o->road_height + 0.008f;
  for (double along = 9.0 + (double)(seed % 17); along < length - 6.0;
       along += 36.0 + (double)((seed >> 12) % 14)) {
    seed ^= seed >> 12, seed ^= seed << 25, seed ^= seed >> 27;
    size_t segment;
    local_point_t on_line = along_polyline(points, count, along, &segment);
    vec2_t lateral = v2_normalize(laterals[segment > 0 ? segment - 1 : 0]);
    size_t at_point = segment < count ? segment : count - 1;
    double side = ((seed >> 20) & 1) ? 0.5 : -0.5;
    local_point_t centre =
        offset(on_line, lateral,
               side * halves[at_point] + ((seed >> 30) % 5) * 0.1 - 0.2);
    uint32_t base = (uint32_t)part->mesh.vertex_count;
    vec3_t up = vec3(0, 1, 0);
    if (!add_vertex(&part->mesh, at(centre, y), up, 0.5f, 0.5f)) return false;
    for (int k = 0; k < 12; k++) {
      double angle = k * M_PI / 6.0;
      local_point_t rim = {centre.x + cos(angle) * 0.34,
                           centre.z + sin(angle) * 0.34};
      if (!add_vertex(&part->mesh, at(rim, y), up, 0, 0)) return false;
    }
    for (int k = 0; k < 12; k++)
      if (!add_face(&part->mesh, base, base + 1 + (uint32_t)k,
                    base + 1 + (uint32_t)((k + 1) % 12), up))
        return false;
  }
  if (part->mesh.vertex_count == 0) {
    mesh_data_free(&part->mesh);
    chunk->part_count--;
    return true;
  }
  finish_part(chunk, part);
  return true;
}

static bool generate_road(layout_t* layout, const city_model_t* model,
                          const city_road_t* road, road_fit_t fit,
                          road_cuts_t cuts, city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  local_point_t* cut = malloc(road->point_count * sizeof(local_point_t));
  size_t cut_count = cut ? polyline_cut(road->points, road->point_count,
                                        cuts.start, cuts.end, cut)
                         : 0;
  const double step = road->class == ROAD_CLASS_PATH ? 2.0 : 8.0;
  size_t capacity = road->point_count + 2 +
                    (size_t)(cut ? polyline_length(cut, cut_count) / step : 0);
  local_point_t* points = malloc(capacity * sizeof(local_point_t));
  vec2_t* laterals = malloc(capacity * sizeof(vec2_t));
  double* halves = malloc(capacity * sizeof(double));
  double* pavements = malloc(capacity * sizeof(double));
  double* inner = malloc(capacity * sizeof(double));
  double* outer = malloc(capacity * sizeof(double));
  bool ok = cut && points && laterals && halves && pavements && inner && outer;
  size_t count = ok ? densify(cut, cut_count, step, points) : 0;
  free(cut);
  if (!ok || polyline_length(points, count) < 0.05) { /* nothing to draw */
    free(points);
    free(laterals);
    free(halves);
    free(pavements);
    free(inner);
    free(outer);
    return ok;
  }
  lateral_offsets(points, count, laterals);
  fit_along(model, road, o, fit, points, count, halves, pavements);
  city_chunk_t* chunk = chunk_at(layout, centroid(points, count));
  bool for_people =
      road->class == ROAD_CLASS_PEDESTRIAN || road->class == ROAD_CLASS_PATH;
  bool water = road->class == ROAD_CLASS_WATERWAY;
  /* Streets for people and paths sit above the roads and the squares
   * they cross; streams just above the grass and below everything else,
   * with nothing to collide with. */
  float height = for_people ? o->road_height + 0.02f
                 : water    ? 0.02f
                            : o->road_height;
  city_part_t* part = city_chunk_add_part(
      chunk,
      water        ? "water"
      : for_people ? "paving"
                   : "road",
      water ? PART_COLLIDER_NONE : PART_COLLIDER_MESH, COLLISION_LAYER_GROUND,
      o->road_friction, (uint64_t)road->source_id);
  for (size_t i = 0; i < count; i++)
    inner[i] = -halves[i], outer[i] = halves[i];
  if (road->class == ROAD_CLASS_PATH) {
    /* Only the stretches that are not on a street: runs of consecutive
     * points off the asphalt and pavements, each its own strip. */
    size_t run_start = 0;
    bool in_run = false;
    ok = part != NULL;
    for (size_t i = 0; ok && i <= count; i++) {
      bool outside =
          i < count && !on_a_street(model, road, points[i], o->pavement_width);
      if (outside && !in_run) run_start = i, in_run = true;
      if (!outside && in_run) {
        in_run = false;
        if (i - run_start >= 2)
          ok = add_strip(&part->mesh, points + run_start, laterals + run_start,
                         i - run_start, inner + run_start, outer + run_start,
                         height, 3.0f);
      }
    }
    if (ok && part->mesh.vertex_count == 0) {
      mesh_data_free(&part->mesh); /* a path entirely on streets */
      chunk->part_count--;
      part = NULL;
    }
  } else {
    ok = part && add_strip(&part->mesh, points, laterals, count, inner, outer,
                           height, for_people ? 3.0f : o->road_texture_length);
  }
  if (ok && part) finish_part(chunk, part);
  stats->roads++;

  if (ok && fit.pavement > 0) {
    city_part_t* pavement = city_chunk_add_part(
        chunk, "pavement", PART_COLLIDER_MESH, COLLISION_LAYER_GROUND,
        o->pavement_friction, (uint64_t)road->source_id);
    city_part_t* kerb =
        pavement ? city_chunk_add_part(chunk, "kerb", PART_COLLIDER_NONE, 0, 0,
                                       (uint64_t)road->source_id)
                 : NULL;
    ok = pavement && kerb;
    if (ok) pavement = &chunk->parts[chunk->part_count - 2];
    for (int side = -1; side <= 1 && ok; side += 2) {
      for (size_t i = 0; i < count; i++) {
        double edge = side * halves[i], far = side * (halves[i] + pavements[i]);
        inner[i] = fmin(edge, far), outer[i] = fmax(edge, far);
      }
      ok = add_strip(&pavement->mesh, points, laterals, count, inner, outer,
                     o->kerb_height, o->pavement_width);
      for (size_t i = 0; i < count && ok; i++) inner[i] = side * halves[i];
      ok = ok && add_kerb(&kerb->mesh, points, laterals, count, inner,
                          o->road_height, o->kerb_height, side < 0);
    }
    if (ok) {
      finish_part(chunk, pavement);
      finish_part(chunk, kerb);
      stats->pavements++;
    }
  }
  if (ok && o->lamp_spacing > 0 && road->class != ROAD_CLASS_PATH &&
      road->class != ROAD_CLASS_SERVICE && road->class != ROAD_CLASS_WATERWAY)
    ok = generate_lamps(layout, road, points, laterals, count, halves,
                        pavements, stats);
  if (ok && !for_people && !water)
    ok = generate_manholes(layout, road, points, laterals, count, halves);
  free(points);
  free(laterals);
  free(halves);
  free(pavements);
  free(inner);
  free(outer);
  return ok;
}

/* ---- areas: water, parks, car parks and squares ---- */

static bool add_area_polygon(city_chunk_t* chunk, city_part_t* part,
                             const local_point_t* points, size_t count, float y,
                             float uv_scale) {
  uint32_t* indices = malloc(3 * (count - 2) * sizeof(uint32_t));
  if (!indices) return false;
  bool clean;
  size_t triangles = triangulate_polygon(points, count, indices, &clean);
  uint32_t base = (uint32_t)part->mesh.vertex_count;
  for (size_t i = 0; i < count; i++)
    if (!add_vertex(&part->mesh, at(points[i], y), vec3(0, 1, 0),
                    (float)(points[i].x * uv_scale),
                    (float)(points[i].z * uv_scale))) {
      free(indices);
      return false;
    }
  for (size_t t = 0; t < triangles; t++) {
    local_point_t tri[3] = {points[indices[t * 3]], points[indices[t * 3 + 1]],
                            points[indices[t * 3 + 2]]};
    if (fabs(polygon_signed_area(tri, 3)) < 1e-4) continue;
    if (!add_face(&part->mesh, base + indices[t * 3], base + indices[t * 3 + 1],
                  base + indices[t * 3 + 2], vec3(0, 1, 0))) {
      free(indices);
      return false;
    }
  }
  free(indices);
  finish_part(chunk, part);
  return true;
}

static bool point_inside(local_point_t p, const local_point_t* ring,
                         size_t count) {
  bool inside = false;
  for (size_t i = 0, j = count - 1; i < count; j = i++) {
    bool crosses = (ring[i].z > p.z) != (ring[j].z > p.z);
    if (!crosses) continue;
    double at =
        (ring[j].x - ring[i].x) * (p.z - ring[i].z) / (ring[j].z - ring[i].z) +
        ring[i].x;
    if (p.x < at) inside = !inside;
  }
  return inside;
}

/* Trees on a jittered grid inside a park: a trunk with a collider and a
 * canopy of three boxes. Deterministic from the area's source id. */
/* A tree: a slim trunk and a crown of two overlapping rounded masses, one
 * leaning a little to a side that the seed picks. */
static bool add_tree_geometry(city_part_t* trunks, city_part_t* leaves,
                              local_point_t p, float height, float crown,
                              uint64_t seed) {
  if (!mesh_data_add_cylinder_y(&trunks->mesh, vec3((float)p.x, 0, (float)p.z),
                                0.16f, height * 0.85f, 6))
    return false;
  float lean_x = (float)((seed >> 8) % 100) / 100.0f - 0.5f;
  float lean_z = (float)((seed >> 16) % 100) / 100.0f - 0.5f;
  vec3_t top = vec3((float)p.x, height, (float)p.z);
  return mesh_data_add_ellipsoid(
             &leaves->mesh, vec3_add(top, vec3(0, crown * 0.45f, 0)),
             vec3(crown * 1.15f, crown * 1.0f, crown * 1.15f), 4, 7) &&
         mesh_data_add_ellipsoid(
             &leaves->mesh,
             vec3_add(top, vec3(lean_x * crown * 1.3f, crown * 0.8f,
                                lean_z * crown * 1.3f)),
             vec3(crown * 0.8f, crown * 0.75f, crown * 0.8f), 4, 7);
}

static bool generate_trees(layout_t* layout, const city_area_t* area,
                           city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  if (o->tree_spacing <= 0 || area->point_count < 3) return true;
  double min_x = 1e30, max_x = -1e30, min_z = 1e30, max_z = -1e30;
  for (size_t i = 0; i < area->point_count; i++) {
    min_x = fmin(min_x, area->points[i].x),
    max_x = fmax(max_x, area->points[i].x);
    min_z = fmin(min_z, area->points[i].z),
    max_z = fmax(max_z, area->points[i].z);
  }
  city_chunk_t* chunk =
      chunk_at(layout, centroid(area->points, area->point_count));
  city_part_t* trunks = city_chunk_add_part(chunk, "trunk", PART_COLLIDER_MESH,
                                            COLLISION_LAYER_PROP, 0.5f,
                                            (uint64_t)area->source_id);
  city_part_t* leaves =
      trunks ? city_chunk_add_part(chunk, "leaves", PART_COLLIDER_NONE, 0, 0,
                                   (uint64_t)area->source_id)
             : NULL;
  if (!trunks || !leaves) return false;
  trunks = &chunk->parts[chunk->part_count - 2];
  uint64_t seed = (uint64_t)area->source_id * 0x9e3779b97f4a7c15ull;
  int planted = 0;
  for (double z = min_z + o->tree_spacing * 0.5; z < max_z;
       z += o->tree_spacing) {
    for (double x = min_x + o->tree_spacing * 0.5; x < max_x;
         x += o->tree_spacing) {
      seed ^= seed >> 12, seed ^= seed << 25, seed ^= seed >> 27;
      double jx = ((seed >> 11) % 1000) / 1000.0 - 0.5,
             jz = ((seed >> 31) % 1000) / 1000.0 - 0.5;
      local_point_t p = {x + jx * o->tree_spacing * 0.6,
                         z + jz * o->tree_spacing * 0.6};
      /* Keep the canopy inside the park. */
      local_point_t margin[4] = {{p.x - 2.5, p.z},
                                 {p.x + 2.5, p.z},
                                 {p.x, p.z - 2.5},
                                 {p.x, p.z + 2.5}};
      bool fits = point_inside(p, area->points, area->point_count);
      for (int k = 0; k < 4 && fits; k++)
        fits = point_inside(margin[k], area->points, area->point_count);
      if (!fits) continue;
      float height = 3.2f + (float)((seed >> 40) % 220) / 100.0f;
      float crown = 2.0f + (float)((seed >> 50) % 100) / 100.0f;
      if (!add_tree_geometry(trunks, leaves, p, height, crown, seed))
        return false;
      planted++;
    }
  }
  if (planted == 0) {
    mesh_data_free(&trunks->mesh);
    mesh_data_free(&leaves->mesh);
    chunk->part_count -= 2;
    return true;
  }
  finish_part(chunk, trunks);
  finish_part(chunk, leaves);
  stats->trees += (size_t)planted;
  return true;
}

/* ---- street furniture ---- */

/* The direction of the nearest road to a point, a unit vector, and how far
 * it is. */
static vec2_t nearest_road_direction(const city_model_t* model, local_point_t p,
                                     double* distance) {
  double best = 1e30;
  vec2_t direction = v2(1, 0);
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    if (road->class == ROAD_CLASS_WATERWAY) continue;
    for (size_t k = 1; k < road->point_count; k++) {
      double d =
          point_segment_distance(p, road->points[k - 1], road->points[k]);
      if (d < best) {
        best = d;
        direction = v2_normalize(v2_sub(road->points[k], road->points[k - 1]));
      }
    }
  }
  *distance = best;
  return direction;
}

/* A box turned by `along` (its x axis along it): corners as a rotated box
 * built from a few quads would be heavy; a box mesh with its half-extents
 * swapped for near-axis alignment is plenty for furniture at this size. */
static bool add_oriented_box(mesh_data_t* mesh, vec3_t centre, vec3_t half,
                             vec2_t along) {
  bool x_aligned = fabs(along.x) >= fabs(along.z);
  vec3_t h = x_aligned ? half : vec3(half.z, half.y, half.x);
  return mesh_data_add_box(mesh, centre, h);
}

static bool generate_furniture(layout_t* layout, const city_model_t* model,
                               city_generator_stats_t* stats) {
  (void)stats;
  for (size_t f = 0; f < model->furniture_count; f++) {
    local_point_t p = {model->furniture[f].x, model->furniture[f].z};
    int kind = model->furniture[f].kind;
    double road_distance;
    vec2_t along = nearest_road_direction(model, p, &road_distance);
    city_chunk_t* chunk = chunk_at(layout, p);
    city_part_t* part = city_chunk_add_part(
        chunk, "furniture", PART_COLLIDER_MESH, COLLISION_LAYER_PROP, 0.5f, 0);
    if (!part) return false;
    float y0 = layout->options->kerb_height;
    vec3_t c = vec3((float)p.x, y0, (float)p.z);
    bool ok = true;
    if (kind == OSM_FURNITURE_BENCH) {
      ok = add_oriented_box(&part->mesh, vec3_add(c, vec3(0, 0.45f, 0)),
                            vec3(0.85f, 0.04f, 0.22f), along) &&
           add_oriented_box(&part->mesh, vec3_add(c, vec3(0, 0.78f, 0)),
                            vec3(0.85f, 0.18f, 0.025f), along) &&
           add_oriented_box(&part->mesh, vec3_add(c, vec3(0, 0.22f, 0)),
                            vec3(0.78f, 0.22f, 0.18f), along);
    } else if (kind == OSM_FURNITURE_BUS_STOP) {
      ok = mesh_data_add_cylinder_y(&part->mesh, c, 0.04f, 2.6f, 6) &&
           add_oriented_box(&part->mesh, vec3_add(c, vec3(0, 2.5f, 0)),
                            vec3(0.02f, 0.2f, 0.28f), along) &&
           add_oriented_box(&part->mesh,
                            vec3_add(c, vec3(1.3f * (float)along.x, 2.5f,
                                             1.3f * (float)along.z)),
                            vec3(1.5f, 0.04f, 0.8f), along) &&
           mesh_data_add_cylinder_y(&part->mesh,
                                    vec3_add(c, vec3(2.6f * (float)along.x, 0,
                                                     2.6f * (float)along.z)),
                                    0.04f, 2.5f, 6);
    } else if (kind == OSM_FURNITURE_BOLLARD) {
      ok = mesh_data_add_cylinder_y(&part->mesh, c, 0.1f, 0.85f, 8);
    } else if (kind == OSM_FURNITURE_SIGNALS) {
      ok = mesh_data_add_cylinder_y(&part->mesh, c, 0.06f, 3.4f, 6) &&
           add_oriented_box(&part->mesh, vec3_add(c, vec3(0, 3.1f, 0)),
                            vec3(0.17f, 0.45f, 0.17f), along);
      city_part_t* red = ok ? city_chunk_add_part(chunk, "signal_red",
                                                  PART_COLLIDER_NONE, 0, 0, 0)
                            : NULL;
      city_part_t* amber =
          red ? city_chunk_add_part(chunk, "signal_amber", PART_COLLIDER_NONE,
                                    0, 0, 0)
              : NULL;
      city_part_t* green =
          amber ? city_chunk_add_part(chunk, "signal_green", PART_COLLIDER_NONE,
                                      0, 0, 0)
                : NULL;
      if (!green) return false;
      red = &chunk->parts[chunk->part_count - 3];
      amber = &chunk->parts[chunk->part_count - 2];
      green = &chunk->parts[chunk->part_count - 1];
      part = &chunk->parts[chunk->part_count - 4];
      /* Three lenses on each of the two faces looking along the road. */
      for (int face = -1; face <= 1; face += 2) {
        vec3_t out = vec3(face * (float)along.x * 0.19f, 0,
                          face * (float)along.z * 0.19f);
        vec3_t lens = vec3(0.11f * (float)(fabs(along.z) + 0.2), 0.11f,
                           0.11f * (float)(fabs(along.x) + 0.2));
        if (!mesh_data_add_box(&red->mesh,
                               vec3_add(vec3_add(c, vec3(0, 3.35f, 0)), out),
                               lens) ||
            !mesh_data_add_box(&amber->mesh,
                               vec3_add(vec3_add(c, vec3(0, 3.1f, 0)), out),
                               lens) ||
            !mesh_data_add_box(&green->mesh,
                               vec3_add(vec3_add(c, vec3(0, 2.85f, 0)), out),
                               lens))
          return false;
      }
      finish_part(chunk, red);
      finish_part(chunk, amber);
      finish_part(chunk, green);
    }
    if (!ok) return false;
    finish_part(chunk, part);
  }
  return true;
}

/* The mapped single trees, each a trunk and a crown like a park's, with a
 * collider on the trunk. They sit on the ground where they were mapped. */
static bool generate_street_trees(layout_t* layout, const city_model_t* model,
                                  city_generator_stats_t* stats) {
  if (layout->options->tree_spacing <= 0) return true;
  for (size_t t = 0; t < model->tree_count; t++) {
    local_point_t p = model->trees[t];
    city_chunk_t* chunk = chunk_at(layout, p);
    city_part_t* trunks = city_chunk_add_part(
        chunk, "trunk", PART_COLLIDER_MESH, COLLISION_LAYER_PROP, 0.5f, 0);
    city_part_t* leaves =
        trunks
            ? city_chunk_add_part(chunk, "leaves", PART_COLLIDER_NONE, 0, 0, 0)
            : NULL;
    if (!trunks || !leaves) return false;
    trunks = &chunk->parts[chunk->part_count - 2];
    uint64_t seed = (uint64_t)(t + 1) * 0x9e3779b97f4a7c15ull;
    seed ^= seed >> 12, seed ^= seed << 25, seed ^= seed >> 27;
    float height = 3.6f + (float)((seed >> 40) % 220) / 100.0f;
    float crown = 2.1f + (float)((seed >> 50) % 100) / 100.0f;
    if (!add_tree_geometry(trunks, leaves, p, height, crown, seed))
      return false;
    finish_part(chunk, trunks);
    finish_part(chunk, leaves);
    stats->trees++;
  }
  return true;
}

static bool generate_area(layout_t* layout, const city_area_t* area,
                          city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  const char* material = "park";
  float y = 0.025f;
  part_collider_t collider = PART_COLLIDER_NONE;
  float friction = 0;
  switch (area->kind) {
    case AREA_WATER:
      material = "water";
      y = 0.02f;
      break;
    case AREA_PARKING:
      material = "parking";
      y = o->road_height - 0.01f;
      collider = PART_COLLIDER_MESH;
      friction = o->road_friction;
      break;
    case AREA_SQUARE:
      material = "paving";
      y = o->road_height + 0.012f;
      collider = PART_COLLIDER_MESH;
      friction = o->pavement_friction;
      break;
    default:
      break;
  }
  if (area->point_count < 3) return true;
  city_chunk_t* chunk =
      chunk_at(layout, centroid(area->points, area->point_count));
  city_part_t* part =
      city_chunk_add_part(chunk, material, collider, COLLISION_LAYER_GROUND,
                          friction, (uint64_t)area->source_id);
  if (!part) return false;
  if (!add_area_polygon(chunk, part, area->points, area->point_count, y,
                        o->ground_uv_scale))
    return false;
  if (part->mesh.index_count == 0) {
    mesh_data_free(&part->mesh);
    chunk->part_count--;
    return true;
  }
  stats->areas++;
  if (area->kind == AREA_PARK) return generate_trees(layout, area, stats);
  return true;
}

/* Wall quads around a ring. The ring is anticlockwise for an outline and
 * clockwise for a courtyard, so "outside is to the right" faces the
 * street in one case and the courtyard in the other. */
/* A pitched roof over an irregular footprint: the roof is a ridge along
 * the long axis of the footprint's bounding rectangle, falling away on
 * both sides, and the walls rise to meet it. */
typedef struct {
  vec2_t across; /* unit vector across the ridge */
  double mid;    /* across-coordinate of the ridge line */
  double slope;  /* rise per metre away from the ridge */
  double rise;   /* height of the ridge above the eaves */
} ridge_t;

static double ridge_across(const ridge_t* ridge, local_point_t p) {
  return p.x * ridge->across.x + p.z * ridge->across.z;
}

static double ridge_height(const ridge_t* ridge, double eaves,
                           local_point_t p) {
  return eaves + ridge->rise -
         ridge->slope * fabs(ridge_across(ridge, p) - ridge->mid);
}

/* ---- porticoes ---- */

#define ARCADE_DEPTH 3.0
#define ARCADE_PILLAR 0.7
#define ARCADE_SPACING 4.5

typedef struct arcade_edge {
  local_point_t a, b;
} arcade_edge_t;

static bool street_is_arcaded(const city_generator_options_t* o,
                              const char* name) {
  if (!name[0] || !o->arcade_streets[0]) return false;
  size_t length = strlen(name);
  const char* cursor = o->arcade_streets;
  while (*cursor) {
    const char* end = strchr(cursor, ',');
    size_t token = end ? (size_t)(end - cursor) : strlen(cursor);
    if (token == length && strncmp(cursor, name, length) == 0) return true;
    cursor += token + (end ? 1 : 0);
  }
  return false;
}

/* True when edge i of a building faces a named arcaded pedestrian street
 * closely and runs alongside it, on a building deep enough to set a
 * ground floor back. */
static bool edge_faces_arcade(const city_model_t* model,
                              const city_generator_options_t* o,
                              const city_building_t* building, size_t i,
                              double depth_available) {
  if (!o->arcade_streets[0] || depth_available < 8.0) return false;
  local_point_t a = building->points[i];
  local_point_t b = building->points[(i + 1) % building->point_count];
  double length = hypot(b.x - a.x, b.z - a.z);
  if (length < 6.0) return false;
  vec2_t direction = v2_normalize(v2_sub(b, a));
  local_point_t middle = {(a.x + b.x) * 0.5, (a.z + b.z) * 0.5};
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    if (road->class != ROAD_CLASS_PEDESTRIAN ||
        !street_is_arcaded(o, road->name))
      continue;
    for (size_t k = 1; k < road->point_count; k++) {
      vec2_t along = v2_normalize(v2_sub(road->points[k], road->points[k - 1]));
      if (fabs(direction.x * along.x + direction.z * along.z) < 0.9) continue;
      if (point_segment_distance(middle, road->points[k - 1], road->points[k]) <
          road->width * 0.5 + 3.5)
        return true;
    }
  }
  return false;
}

static double building_depth(const city_building_t* building) {
  /* The shorter side of the bounding rectangle. */
  double best = 1e30;
  for (size_t i = 0; i < building->point_count; i++) {
    vec2_t axis =
        v2_normalize(v2_sub(building->points[(i + 1) % building->point_count],
                            building->points[i]));
    if (v2_length(axis) < 0.5) continue;
    double lo = 1e30, hi = -1e30, lo2 = 1e30, hi2 = -1e30;
    for (size_t k = 0; k < building->point_count; k++) {
      double along =
          building->points[k].x * axis.x + building->points[k].z * axis.z;
      double across =
          -building->points[k].x * axis.z + building->points[k].z * axis.x;
      lo = fmin(lo, along), hi = fmax(hi, along);
      lo2 = fmin(lo2, across), hi2 = fmax(hi2, across);
    }
    best = fmin(best, fmin(hi - lo, hi2 - lo2));
  }
  return best;
}

static bool arcade_continues_at(const layout_t* layout, local_point_t p) {
  int touching = 0;
  for (size_t e = 0; e < layout->arcade_edge_count; e++) {
    const arcade_edge_t* edge = &layout->arcade_edges[e];
    if (hypot(edge->a.x - p.x, edge->a.z - p.z) < 1.5) touching++;
    if (hypot(edge->b.x - p.x, edge->b.z - p.z) < 1.5) touching++;
  }
  return touching >= 2; /* this edge and its neighbour */
}

static bool add_wall_quad(city_part_t* walls, local_point_t p, local_point_t q,
                          float y0, float y1, vec3_t normal, float u0, float u1,
                          float v0, float v1) {
  uint32_t base = (uint32_t)walls->mesh.vertex_count;
  if (!add_vertex(&walls->mesh, at(p, y0), normal, u0, v0) ||
      !add_vertex(&walls->mesh, at(q, y0), normal, u1, v0) ||
      !add_vertex(&walls->mesh, at(q, y1), normal, u1, v1) ||
      !add_vertex(&walls->mesh, at(p, y1), normal, u0, v1))
    return false;
  return add_face(&walls->mesh, base, base + 1, base + 2, normal) &&
         add_face(&walls->mesh, base, base + 2, base + 3, normal);
}

/* One square pillar standing at `centre`, aligned to the edge. */
static bool add_pillar(city_part_t* walls, local_point_t centre, vec2_t along,
                       float y0, float y1, float storey) {
  double h = ARCADE_PILLAR * 0.5;
  vec2_t out = v2(-along.z, along.x);
  local_point_t corner[4];
  double signs[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
  for (int k = 0; k < 4; k++) {
    corner[k].x =
        centre.x + along.x * h * signs[k][0] + out.x * h * signs[k][1];
    corner[k].z =
        centre.z + along.z * h * signs[k][0] + out.z * h * signs[k][1];
  }
  for (int k = 0; k < 4; k++) {
    local_point_t p = corner[k], q = corner[(k + 1) % 4];
    vec2_t d = v2_normalize(v2_sub(q, p));
    vec3_t normal = vec3((float)-d.z, 0, (float)d.x);
    /* The corners run anticlockwise seen from above: signs above go
     * (-,-), (+,-), (+,+), (-,+). */
    if (!add_wall_quad(walls, p, q, y0, y1, normal, 0, 0.25f, y0 / storey,
                       y1 / storey))
      return false;
  }
  return true;
}

/* The ground floor of an edge set back behind pillars: the wall at the
 * back, the returns at the ends unless the arcade carries on, the soffit
 * and the pillars. `floor` collects the paved walkway. */
static bool add_portico(const layout_t* layout, city_part_t* walls,
                        city_part_t* walkway, local_point_t a, local_point_t b,
                        float height, float skirt) {
  const city_generator_options_t* o = layout->options;
  vec2_t along = v2_sub(b, a);
  double length = v2_length(along);
  vec2_t direction = v2_normalize(along);
  vec3_t outward = vec3((float)-direction.z, 0, (float)direction.x);
  vec2_t inward = v2(direction.z, -direction.x);
  local_point_t a2 = offset(a, inward, ARCADE_DEPTH);
  local_point_t b2 = offset(b, inward, ARCADE_DEPTH);
  float storey = o->storey_height;
  float top = fminf(height, 1.5f * storey);
  float u1 = (float)(length / o->window_width);
  /* The back wall faces out toward the street. */
  if (!add_wall_quad(walls, a2, b2, -skirt, top, outward, 0, u1,
                     -skirt / storey, top / storey))
    return false;
  /* Returns close the ends of an arcade that stops there. */
  if (!arcade_continues_at(layout, a) &&
      !add_wall_quad(walls, a, a2, -skirt, top,
                     vec3((float)direction.x, 0, (float)direction.z), 0,
                     (float)(ARCADE_DEPTH / o->window_width), -skirt / storey,
                     top / storey))
    return false;
  if (!arcade_continues_at(layout, b) &&
      !add_wall_quad(walls, b2, b, -skirt, top,
                     vec3((float)-direction.x, 0, (float)-direction.z), 0,
                     (float)(ARCADE_DEPTH / o->window_width), -skirt / storey,
                     top / storey))
    return false;
  /* The soffit overhead, facing down. */
  uint32_t base = (uint32_t)walls->mesh.vertex_count;
  vec3_t down = vec3(0, -1, 0);
  if (!add_vertex(&walls->mesh, at(a, top), down, 0, 0) ||
      !add_vertex(&walls->mesh, at(b, top), down, u1, 0) ||
      !add_vertex(&walls->mesh, at(b2, top), down, u1, 1) ||
      !add_vertex(&walls->mesh, at(a2, top), down, 0, 1))
    return false;
  if (!add_face(&walls->mesh, base, base + 1, base + 2, down) ||
      !add_face(&walls->mesh, base, base + 2, base + 3, down))
    return false;
  /* Pillars along the building line, a face in from it so the upper
   * walls overhang them. */
  int pillars = (int)floor(length / ARCADE_SPACING + 0.5);
  if (pillars < 1) pillars = 1;
  for (int k = 0; k < pillars; k++) {
    double t = (k + 0.5) / pillars * length;
    local_point_t centre =
        offset(offset(a, direction, t), inward, ARCADE_PILLAR * 0.5);
    if (!add_pillar(walls, centre, direction, -skirt, top, storey))
      return false;
  }
  /* Round arches between the pillars: a spandrel panel on the building line
   * from the curve up to the soffit, facing the street. */
  float spring = fminf(top - 2.2f, 2.5f);
  float radius = 1.9f;
  for (int k = 0; k + 1 < pillars; k++) {
    double t = (k + 1.0) / pillars * length;
    double centre_along = t;
    for (int seg = 0; seg < 8; seg++) {
      double a0 = M_PI * seg / 8.0, a1 = M_PI * (seg + 1) / 8.0;
      local_point_t p0 = offset(a, direction, centre_along + cos(a0) * radius);
      local_point_t p1 = offset(a, direction, centre_along + cos(a1) * radius);
      p0 = offset(p0, inward, ARCADE_PILLAR * 0.5);
      p1 = offset(p1, inward, ARCADE_PILLAR * 0.5);
      float y0 = spring + (float)sin(a0) * radius,
            y1 = spring + (float)sin(a1) * radius;
      if (y0 > top - 0.05f) y0 = top - 0.05f;
      if (y1 > top - 0.05f) y1 = top - 0.05f;
      uint32_t base = (uint32_t)walls->mesh.vertex_count;
      float u0 = (float)((centre_along + cos(a0) * radius) / o->window_width);
      float u1v = (float)((centre_along + cos(a1) * radius) / o->window_width);
      if (!add_vertex(&walls->mesh, at(p0, y0), outward, u0, y0 / storey) ||
          !add_vertex(&walls->mesh, at(p1, y1), outward, u1v, y1 / storey) ||
          !add_vertex(&walls->mesh, at(p1, top), outward, u1v, top / storey) ||
          !add_vertex(&walls->mesh, at(p0, top), outward, u0, top / storey))
        return false;
      /* Seen from the street (a to b to the left of it): wound to face out. */
      if (!add_face(&walls->mesh, base, base + 1, base + 2, outward) ||
          !add_face(&walls->mesh, base, base + 2, base + 3, outward))
        return false;
    }
  }
  /* The paved walkway under it. */
  float y = o->road_height + 0.02f;
  uint32_t f = (uint32_t)walkway->mesh.vertex_count;
  vec3_t up = vec3(0, 1, 0);
  if (!add_vertex(&walkway->mesh, at(a, y), up, 0, 0) ||
      !add_vertex(&walkway->mesh, at(b, y), up, u1, 0) ||
      !add_vertex(&walkway->mesh, at(b2, y), up, u1, 1) ||
      !add_vertex(&walkway->mesh, at(a2, y), up, 0, 1))
    return false;
  return add_face(&walkway->mesh, f, f + 1, f + 2, up) &&
         add_face(&walkway->mesh, f, f + 2, f + 3, up);
}

/* True when edge i of a building runs alongside a main street or a
 * pedestrian street close by and the building is tall enough for a shop
 * on its ground floor; most buildings on such a street have one. */
static bool edge_faces_shops(const city_model_t* model,
                             const city_building_t* building, size_t i) {
  if (building->height < 6.0f) return false;
  unsigned hash = (unsigned)((uint64_t)building->source_id * 2654435761u >> 9);
  if (hash % 100 >= 65) return false;
  local_point_t a = building->points[i];
  local_point_t b = building->points[(i + 1) % building->point_count];
  if (hypot(b.x - a.x, b.z - a.z) < 5.0) return false;
  vec2_t direction = v2_normalize(v2_sub(b, a));
  local_point_t middle = {(a.x + b.x) * 0.5, (a.z + b.z) * 0.5};
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    bool main_street = road->class == ROAD_CLASS_PRIMARY ||
                       road->class == ROAD_CLASS_SECONDARY ||
                       road->class == ROAD_CLASS_TERTIARY ||
                       road->class == ROAD_CLASS_PEDESTRIAN;
    if (!main_street) continue;
    for (size_t k = 1; k < road->point_count; k++) {
      vec2_t along = v2_normalize(v2_sub(road->points[k], road->points[k - 1]));
      if (fabs(direction.x * along.x + direction.z * along.z) < 0.9) continue;
      if (point_segment_distance(middle, road->points[k - 1], road->points[k]) <
          road->width * 0.5 + 7.5)
        return true;
    }
  }
  return false;
}

static bool add_walls(const layout_t* layout, city_part_t* walls,
                      city_part_t* walkway, city_part_t* shops,
                      const local_point_t* ring, size_t count, float h,
                      const ridge_t* ridge, float skirt, const bool* arcaded,
                      const bool* shop) {
  const city_generator_options_t* o = layout->options;
  for (size_t i = 0; i < count; i++) {
    local_point_t a = ring[i], b = ring[(i + 1) % count];
    vec2_t along = v2_sub(b, a);
    double length = v2_length(along);
    if (length < 1e-6) continue;
    vec2_t direction = v2_normalize(along);
    vec3_t normal = vec3((float)-direction.z, 0, (float)direction.x);
    bool portico = arcaded && arcaded[i];
    bool shopfront = !portico && shop && shop[i];
    float storey = o->storey_height;
    float bottom = portico ? fminf(h, 1.5f * storey)
                           : (shopfront ? fminf(h, storey) : -skirt);
    /* Split where the edge crosses the ridge so the top follows it. */
    local_point_t pieces[3] = {a, b, b};
    size_t piece_count = 2;
    if (ridge) {
      double ca = ridge_across(ridge, a) - ridge->mid;
      double cb = ridge_across(ridge, b) - ridge->mid;
      if (ca * cb < 0) {
        double t = ca / (ca - cb);
        local_point_t crossing = {a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t};
        pieces[1] = crossing;
        pieces[2] = b;
        piece_count = 3;
      }
    }
    double travelled = 0;
    for (size_t k = 0; k + 1 < piece_count; k++) {
      local_point_t p = pieces[k], q = pieces[k + 1];
      double span = hypot(q.x - p.x, q.z - p.z);
      float top_p = ridge ? (float)ridge_height(ridge, h, p) : h;
      float top_q = ridge ? (float)ridge_height(ridge, h, q) : h;
      float u0 = (float)(travelled / o->window_width);
      float u1 = (float)((travelled + span) / o->window_width);
      uint32_t base = (uint32_t)walls->mesh.vertex_count;
      /* Above a portico the texture restarts at a clean storey line. */
      float v_bottom =
          portico ? 0.0f : (shopfront ? bottom / storey : -skirt / storey);
      float v_shift = portico ? bottom / storey : 0.0f;
      if (shopfront && !add_wall_quad(shops, p, q, -skirt, bottom, normal,
                                      (float)(travelled / 4.0),
                                      (float)((travelled + span) / 4.0),
                                      -skirt / storey, bottom / storey))
        return false;
      if (!add_vertex(&walls->mesh, at(p, bottom), normal, u0, v_bottom) ||
          !add_vertex(&walls->mesh, at(q, bottom), normal, u1, v_bottom) ||
          !add_vertex(&walls->mesh, at(q, top_q), normal, u1,
                      top_q / storey - v_shift) ||
          !add_vertex(&walls->mesh, at(p, top_p), normal, u0,
                      top_p / storey - v_shift))
        return false;
      if (!add_face(&walls->mesh, base, base + 1, base + 2, normal) ||
          !add_face(&walls->mesh, base, base + 2, base + 3, normal))
        return false;
      travelled += span;
    }
    if (portico && !add_portico(layout, walls, walkway, a, b, h, skirt))
      return false;
  }
  return true;
}

/* The smallest rectangle around a footprint, found by trying every edge
 * direction. `axis` runs along the long side. */
typedef struct {
  vec2_t axis;
  vec2_t across;
  double along_min, along_max, across_min, across_max;
  double area;
} bounding_rectangle_t;

static bounding_rectangle_t bounding_rectangle(const local_point_t* points,
                                               size_t count) {
  bounding_rectangle_t best;
  best.area = 1e300;
  for (size_t i = 0; i < count; i++) {
    vec2_t axis = v2_normalize(v2_sub(points[(i + 1) % count], points[i]));
    if (v2_length(axis) < 0.5) continue;
    vec2_t across = v2_right(axis);
    double a0 = 1e300, a1 = -1e300, c0 = 1e300, c1 = -1e300;
    for (size_t k = 0; k < count; k++) {
      double a = points[k].x * axis.x + points[k].z * axis.z;
      double c = points[k].x * across.x + points[k].z * across.z;
      a0 = fmin(a0, a), a1 = fmax(a1, a), c0 = fmin(c0, c), c1 = fmax(c1, c);
    }
    double area = (a1 - a0) * (c1 - c0);
    if (area < best.area) {
      best.axis = axis, best.across = across;
      best.along_min = a0, best.along_max = a1, best.across_min = c0,
      best.across_max = c1;
      best.area = area;
    }
  }
  if (best.along_max - best.along_min < best.across_max - best.across_min) {
    /* Swap so the axis follows the long side. */
    bounding_rectangle_t swapped = best;
    swapped.axis = best.across, swapped.across = v2_right(best.across);
    swapped.along_min = best.across_min, swapped.along_max = best.across_max;
    swapped.across_min = -best.along_max, swapped.across_max = -best.along_min;
    best = swapped;
  }
  return best;
}

static vec3_t rectangle_point(const bounding_rectangle_t* r, double along,
                              double across, float y) {
  return vec3((float)(r->axis.x * along + r->across.x * across), y,
              (float)(r->axis.z * along + r->across.z * across));
}

typedef enum { ROOF_FLAT, ROOF_HIPPED, ROOF_GABLED, ROOF_RIDGE } roof_shape_t;

/* Footprints that nearly fill their bounding rectangle and have no
 * courtyard get a pitched roof with eaves; a narrow, long one a gabled
 * roof. Every other building under 22 m, over a hut and under a
 * warehouse, gets a ridge roof that follows its outline. Taller and
 * larger ones stay flat. */
static roof_shape_t roof_shape(const city_building_t* building) {
  double footprint =
      fabs(polygon_signed_area(building->points, building->point_count));
  bool nearly_rectangular = false;
  roof_shape_t shape = ROOF_FLAT;
  if (building->hole_count == 0 && building->point_count >= 4) {
    bounding_rectangle_t r =
        bounding_rectangle(building->points, building->point_count);
    if (r.area > 0 && footprint / r.area >= 0.82) {
      nearly_rectangular = true;
      double width = r.across_max - r.across_min;
      double length = r.along_max - r.along_min;
      shape = width < 9.0 && length > 1.6 * width ? ROOF_GABLED : ROOF_HIPPED;
    }
  }
  if (nearly_rectangular) return shape;
  if (building->height < 22.0f && footprint > 25.0 && footprint < 9000.0)
    return ROOF_RIDGE;
  return ROOF_FLAT;
}

/* A pitched roof with eaves on footprints that nearly fill their bounding
 * rectangle and have no courtyard: hipped, or gabled when the footprint is
 * a narrow terrace, with the gable triangles added to the walls. Sets
 * *done when it built one. */
static bool add_pitched_roof(city_part_t* roof, city_part_t* walls,
                             const city_building_t* building, float h,
                             const city_generator_options_t* o, bool* done) {
  *done = false;
  if (building->hole_count > 0 || building->point_count < 4) return true;
  bounding_rectangle_t r =
      bounding_rectangle(building->points, building->point_count);
  double footprint =
      fabs(polygon_signed_area(building->points, building->point_count));
  if (r.area <= 0 || footprint / r.area < 0.82) return true;
  const double eaves = 0.35;
  double width = r.across_max - r.across_min;
  double length = r.along_max - r.along_min;
  bool gabled = width < 9.0 && length > 1.6 * width;
  double a0 = r.along_min - (gabled ? 0 : eaves);
  double a1 = r.along_max + (gabled ? 0 : eaves);
  double c0 = r.across_min - eaves, c1 = r.across_max + eaves;
  double half_span = (c1 - c0) * 0.5, mid = (c0 + c1) * 0.5;
  double inset = gabled ? 0 : fmin(half_span, (a1 - a0) * 0.5);
  /* A tall slender block is a tower: it ends in a spire, not a cap. */
  bool tower = h > 30.0f && width < 16.0 && length < 16.0;
  float rise = tower ? (float)(width * 1.6) : (float)fmin(half_span * 0.6, 4.0);
  float ridge_y = h + rise;
  vec3_t e00 = rectangle_point(&r, a0, c0, h),
         e10 = rectangle_point(&r, a1, c0, h);
  vec3_t e11 = rectangle_point(&r, a1, c1, h),
         e01 = rectangle_point(&r, a0, c1, h);
  vec3_t r0 = rectangle_point(&r, a0 + inset, mid, ridge_y);
  vec3_t r1 = rectangle_point(&r, a1 - inset, mid, ridge_y);
  float s = 0.4f;
  uint32_t base = (uint32_t)roof->mesh.vertex_count;
  vec3_t corners[6] = {e00, e10, e11, e01, r0, r1};
  double us[6] = {a0, a1, a1, a0, a0 + inset, a1 - inset};
  double vs[6] = {c0, c0, c1, c1, mid, mid};
  for (int i = 0; i < 6; i++)
    if (!add_vertex(&roof->mesh, corners[i], vec3(0, 1, 0), (float)(us[i] * s),
                    (float)(vs[i] * s)))
      return false;
  vec3_t up = vec3(0, 1, 0);
  /* Two long slopes, two hips, and the underside of the eaves facing down. */
  if (!add_face(&roof->mesh, base + 0, base + 1, base + 5, up) ||
      !add_face(&roof->mesh, base + 0, base + 5, base + 4, up) ||
      !add_face(&roof->mesh, base + 2, base + 3, base + 4, up) ||
      !add_face(&roof->mesh, base + 2, base + 4, base + 5, up))
    return false;
  if (!gabled && (!add_face(&roof->mesh, base + 1, base + 2, base + 5, up) ||
                  !add_face(&roof->mesh, base + 3, base + 0, base + 4, up)))
    return false;
  if (gabled) {
    /* The gable triangles stand on the end walls, facing out along the
     * axis, textured like the wall below. */
    vec3_t ends[2][3] = {{e10, e11, r1}, {e01, e00, r0}};
    for (int end = 0; end < 2; end++) {
      vec3_t outward = vec3((float)r.axis.x, 0, (float)r.axis.z);
      if (end == 1) outward = vec3_scale(outward, -1);
      uint32_t g = (uint32_t)walls->mesh.vertex_count;
      for (int i = 0; i < 3; i++) {
        float u = (float)(((i == 0)   ? c0
                           : (i == 1) ? c1
                                      : mid) /
                          o->window_width);
        float v = (ends[end][i].y) / o->storey_height;
        if (!add_vertex(&walls->mesh, ends[end][i], outward, u, v))
          return false;
      }
      if (!add_face(&walls->mesh, g, g + 1, g + 2, outward)) return false;
    }
  }
  uint32_t under = (uint32_t)roof->mesh.vertex_count;
  vec3_t down = vec3(0, -1, 0);
  for (int i = 0; i < 4; i++)
    if (!add_vertex(&roof->mesh, corners[i], down, (float)(us[i] * s),
                    (float)(vs[i] * s)))
      return false;
  if (!add_face(&roof->mesh, under + 0, under + 1, under + 2, down) ||
      !add_face(&roof->mesh, under + 0, under + 2, under + 3, down))
    return false;
  *done = true;
  return true;
}

/* One roof triangle over the footprint, cut along the ridge so the two
 * slopes meet in a crisp line, lifted onto the roof surface. */
static bool add_ridge_face(city_part_t* roof, const local_point_t* poly,
                           size_t count, float eaves, const ridge_t* ridge) {
  for (size_t i = 1; i + 1 < count; i++) {
    local_point_t corners[3] = {poly[0], poly[i], poly[i + 1]};
    if (fabs(polygon_signed_area(corners, 3)) < 1e-7) continue;
    uint32_t base = (uint32_t)roof->mesh.vertex_count;
    for (int k = 0; k < 3; k++) {
      double c = ridge_across(ridge, corners[k]) - ridge->mid;
      double sign = c < 0 ? -1.0 : 1.0;
      /* A corner on the ridge takes the side its triangle lies on. */
      if (fabs(c) < 1e-6) {
        double sum = 0;
        for (int m = 0; m < 3; m++)
          sum += ridge_across(ridge, corners[m]) - ridge->mid;
        sign = sum < 0 ? -1.0 : 1.0;
      }
      vec3_t normal = vec3_normalize(
          vec3((float)(sign * ridge->slope * ridge->across.x), 1.0f,
               (float)(sign * ridge->slope * ridge->across.z)));
      float y = (float)ridge_height(ridge, eaves, corners[k]);
      if (!add_vertex(&roof->mesh, at(corners[k], y), normal,
                      (float)(corners[k].x * 0.4), (float)(corners[k].z * 0.4)))
        return false;
    }
    if (!add_face(&roof->mesh, base, base + 1, base + 2, vec3(0, 1, 0)))
      return false;
  }
  return true;
}

static bool add_ridge_triangle(city_part_t* roof, const local_point_t* tri,
                               float eaves, const ridge_t* ridge) {
  double side[3];
  bool crosses = false;
  for (int k = 0; k < 3; k++) {
    side[k] = ridge_across(ridge, tri[k]) - ridge->mid;
  }
  for (int k = 0; k < 3; k++)
    if (side[k] * side[(k + 1) % 3] < -1e-9) crosses = true;
  if (!crosses) return add_ridge_face(roof, tri, 3, eaves, ridge);
  /* Clip the triangle against the ridge line into a negative-side and a
   * positive-side polygon (each of three or four corners). */
  local_point_t negative[4], positive[4];
  size_t nn = 0, np = 0;
  for (int k = 0; k < 3; k++) {
    local_point_t a = tri[k], b = tri[(k + 1) % 3];
    double sa = side[k], sb = side[(k + 1) % 3];
    if (sa <= 0) negative[nn++] = a;
    if (sa >= 0) positive[np++] = a;
    if (sa * sb < -1e-9) {
      double t = sa / (sa - sb);
      local_point_t m = {a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t};
      negative[nn++] = m;
      positive[np++] = m;
    }
  }
  return add_ridge_face(roof, negative, nn, eaves, ridge) &&
         add_ridge_face(roof, positive, np, eaves, ridge);
}

static double terrain_at(const layout_t* layout, double x, double z) {
  return layout->terrain ? city_terrain_height(layout->terrain, x, z) : 0.0;
}

static void lift_part(city_part_t* part, float dy) {
  for (size_t v = 0; v < part->mesh.vertex_count; v++)
    part->mesh.vertices[v].position.y += dy;
}

/* A building stands level at the ground height of its centre. */
static void place_on_terrain(const layout_t* layout,
                             const city_building_t* building,
                             city_part_t* walls, city_part_t* roof) {
  if (!layout->terrain || layout->terrain->cols == 0) return;
  local_point_t c = centroid(building->points, building->point_count);
  float base = (float)terrain_at(layout, c.x, c.z);
  lift_part(walls, base);
  lift_part(roof, base);
}

/* Puts the finished building on the ground and keeps its parts. The
 * walkway part is dropped when no edge needed one. */
static void finish_building(layout_t* layout, const city_building_t* building,
                            city_chunk_t* chunk, city_part_t* walls,
                            city_part_t* roof) {
  place_on_terrain(layout, building, walls, roof);
  finish_part(chunk, walls);
  finish_part(chunk, roof);
  /* The walkway and the shop fronts are the last two parts; keep the ones
   * that have anything in them. */
  size_t n = chunk->part_count;
  city_part_t* walkway = &chunk->parts[n - 2];
  city_part_t* shops = &chunk->parts[n - 1];
  if (shops->mesh.vertex_count == 0) {
    mesh_data_free(&shops->mesh);
    chunk->part_count--;
  } else {
    float base = 0;
    if (layout->terrain && layout->terrain->cols > 0) {
      local_point_t c = centroid(building->points, building->point_count);
      base = (float)terrain_at(layout, c.x, c.z);
    }
    for (size_t v = 0; v < shops->mesh.vertex_count; v++)
      shops->mesh.vertices[v].position.y += base;
    finish_part(chunk, shops);
  }
  if (walkway->mesh.vertex_count == 0) {
    mesh_data_free(&walkway->mesh);
    /* With shops they move down into its place; without, it was the last
     * part left. */
    if (chunk->part_count == n) chunk->parts[n - 2] = chunk->parts[n - 1];
    chunk->part_count--;
  } else {
    finish_part(chunk, walkway);
  }
}

/* ---- landmarks ---- */

typedef enum { LANDMARK_NONE, LANDMARK_TOWN_HALL, LANDMARK_CHURCH } landmark_t;

static bool name_has(const char* name, const char* word) {
  size_t n = strlen(word);
  for (const char* c = name; *c; c++) {
    size_t k = 0;
    while (k < n && c[k] && tolower((unsigned char)c[k]) == word[k]) k++;
    if (k == n) return true;
  }
  return false;
}

static landmark_t landmark_for(const city_building_t* building) {
  if (name_has(building->name, "municipio")) return LANDMARK_TOWN_HALL;
  if (name_has(building->name, "chiesa") || name_has(building->name, "duomo"))
    return LANDMARK_CHURCH;
  return LANDMARK_NONE;
}

/* A square tower from the ground to `height`, ending in a four-sided spire
 * that rises `spire` further. Walls go into `walls`, the spire into `roof`. */
static bool add_tower(city_part_t* walls, city_part_t* roof,
                      local_point_t centre, vec2_t axis, double half,
                      float height, float spire, float skirt,
                      const city_generator_options_t* o) {
  vec2_t across = v2(-axis.z, axis.x);
  local_point_t corner[4];
  double signs[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
  for (int k = 0; k < 4; k++) {
    corner[k].x =
        centre.x + axis.x * half * signs[k][0] + across.x * half * signs[k][1];
    corner[k].z =
        centre.z + axis.z * half * signs[k][0] + across.z * half * signs[k][1];
  }
  for (int k = 0; k < 4; k++) {
    local_point_t p = corner[k], q = corner[(k + 1) % 4];
    vec2_t d = v2_normalize(v2_sub(q, p));
    vec3_t normal = vec3((float)-d.z, 0, (float)d.x);
    float u = (float)(2 * half / o->window_width);
    if (!add_wall_quad(walls, p, q, -skirt, height, normal, 0, u,
                       -skirt / o->storey_height, height / o->storey_height))
      return false;
  }
  /* The spire: a pyramid from slightly overhanging eaves to the apex. */
  vec3_t apex = vec3((float)centre.x, height + spire, (float)centre.z);
  for (int k = 0; k < 4; k++) {
    local_point_t p = corner[k], q = corner[(k + 1) % 4];
    local_point_t mid = {(p.x + q.x) * 0.5, (p.z + q.z) * 0.5};
    vec3_t normal = vec3_normalize(vec3((float)(mid.x - centre.x),
                                        spire / (float)half * 0.5f,
                                        (float)(mid.z - centre.z)));
    uint32_t base = (uint32_t)roof->mesh.vertex_count;
    if (!add_vertex(&roof->mesh, at(p, height), normal, 0, 0) ||
        !add_vertex(&roof->mesh, at(q, height), normal, 1, 0) ||
        !add_vertex(&roof->mesh, apex, normal, 0.5f, 1))
      return false;
    if (!add_face(&roof->mesh, base, base + 1, base + 2, normal)) return false;
  }
  return true;
}

/* The nearest road distance from a point, or a large number. */
static double distance_to_roads(const city_model_t* model, local_point_t p) {
  double nearest = 1e30;
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    if (road->class == ROAD_CLASS_WATERWAY) continue;
    for (size_t k = 1; k < road->point_count; k++)
      nearest = fmin(nearest, point_segment_distance(p, road->points[k - 1],
                                                     road->points[k]));
  }
  return nearest;
}

/* Adds the landmark's tower to the building's parts, returning the centre
 * and the top height of its clock (or 0 for none) through the outputs. */
static bool add_landmark(layout_t* layout, const city_building_t* building,
                         city_part_t* walls, city_part_t* roof, float skirt,
                         landmark_t kind, local_point_t* tower_centre,
                         vec2_t* tower_axis, float* clock_height) {
  const city_generator_options_t* o = layout->options;
  bounding_rectangle_t r =
      bounding_rectangle(building->points, building->point_count);
  double mid_along = 0.5 * (r.along_min + r.along_max);
  double mid_across = 0.5 * (r.across_min + r.across_max);
  double half = kind == LANDMARK_TOWN_HALL ? 3.2 : 2.7;
  /* The side or end that faces the nearer street. */
  local_point_t a, b;
  double inset = half + 0.3;
  if (kind == LANDMARK_TOWN_HALL) {
    a = (local_point_t){
        r.axis.x * mid_along + r.across.x * (r.across_min + inset),
        r.axis.z * mid_along + r.across.z * (r.across_min + inset)};
    b = (local_point_t){
        r.axis.x * mid_along + r.across.x * (r.across_max - inset),
        r.axis.z * mid_along + r.across.z * (r.across_max - inset)};
  } else {
    a = (local_point_t){
        r.axis.x * (r.along_min + inset) + r.across.x * mid_across,
        r.axis.z * (r.along_min + inset) + r.across.z * mid_across};
    b = (local_point_t){
        r.axis.x * (r.along_max - inset) + r.across.x * mid_across,
        r.axis.z * (r.along_max - inset) + r.across.z * mid_across};
  }
  local_point_t centre =
      distance_to_roads(layout->model, a) <= distance_to_roads(layout->model, b)
          ? a
          : b;
  float top = kind == LANDMARK_TOWN_HALL ? 26.0f : 30.0f;
  float spire = kind == LANDMARK_TOWN_HALL ? 7.0f : 9.0f;
  *tower_centre = centre;
  *tower_axis = r.axis;
  *clock_height = kind == LANDMARK_TOWN_HALL ? top - 4.5f : 0.0f;
  return add_tower(walls, roof, centre, r.axis, half, top, spire, skirt, o);
}

/* Four clock faces, one a side, drawn glowing like the lamps at night. */
static bool add_clock_faces(layout_t* layout, const city_building_t* building,
                            city_chunk_t* chunk, local_point_t centre,
                            vec2_t axis, float height) {
  city_part_t* clock =
      city_chunk_add_part(chunk, "lamp_light", PART_COLLIDER_NONE, 0, 0,
                          (uint64_t)building->source_id);
  if (!clock) return false;
  double half = 3.2 + 0.03;
  vec2_t across = v2(-axis.z, axis.x);
  for (int k = 0; k < 4; k++) {
    double angle = k * M_PI * 0.5;
    vec2_t out = v2(axis.x * cos(angle) + across.x * sin(angle),
                    axis.z * cos(angle) + across.z * sin(angle));
    vec2_t side = v2(-out.z, out.x);
    local_point_t face = {centre.x + out.x * half, centre.z + out.z * half};
    local_point_t p = offset(face, side, -1.2), q = offset(face, side, 1.2);
    vec3_t normal = vec3((float)out.x, 0, (float)out.z);
    if (!add_wall_quad(clock, p, q, height - 1.2f, height + 1.2f, normal, 0, 1,
                       0, 1))
      return false;
  }
  float base = layout->terrain ? (float)city_terrain_height(layout->terrain,
                                                            centre.x, centre.z)
                               : 0.0f;
  for (size_t v = 0; v < clock->mesh.vertex_count; v++)
    clock->mesh.vertices[v].position.y += base;
  finish_part(chunk, clock);
  return true;
}

/* Finishes the building's parts, with its landmark tower and clock. */
static bool finish_landmark_building(layout_t* layout,
                                     const city_building_t* building,
                                     city_chunk_t* chunk, city_part_t* walls,
                                     city_part_t* roof, landmark_t landmark,
                                     float skirt) {
  local_point_t centre = {0, 0};
  vec2_t axis = v2(1, 0);
  float clock_height = 0;
  if (landmark != LANDMARK_NONE &&
      !add_landmark(layout, building, walls, roof, skirt, landmark, &centre,
                    &axis, &clock_height))
    return false;
  finish_building(layout, building, chunk, walls, roof);
  if (clock_height > 0 &&
      !add_clock_faces(layout, building, chunk, centre, axis, clock_height))
    return false;
  return true;
}

static bool generate_building(layout_t* layout, const city_building_t* building,
                              city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  size_t count = building->point_count;
  city_chunk_t* chunk = chunk_at(layout, centroid(building->points, count));
  char facade[32];
  unsigned variant = (unsigned)((uint64_t)building->source_id * 2654435761u %
                                (unsigned)o->facade_variants);
  snprintf(facade, sizeof(facade), "facade%u", variant);
  bool on_the_arcade = false;
  for (size_t i = 0; i < count && !on_the_arcade; i++)
    on_the_arcade = edge_faces_arcade(layout->model, o, building, i,
                                      building_depth(building));
  if (on_the_arcade) snprintf(facade, sizeof(facade), "facade_fresco");
  landmark_t landmark = landmark_for(building);
  if (landmark == LANDMARK_TOWN_HALL)
    snprintf(facade, sizeof(facade), "facade_brick");
  city_part_t* walls = city_chunk_add_part(
      chunk, facade, PART_COLLIDER_MESH, COLLISION_LAYER_BUILDING,
      o->building_friction, (uint64_t)building->source_id);
  unsigned roof_hash =
      (unsigned)((uint64_t)building->source_id * 2246822519u >> 7);
  const char* roof_material = roof_shape(building) == ROOF_FLAT ? "roof_flat"
                              : roof_hash % 10 < 7              ? "roof"
                                                                : "roof_slate";
  city_part_t* roof =
      walls
          ? city_chunk_add_part(chunk, roof_material, PART_COLLIDER_MESH,
                                COLLISION_LAYER_BUILDING, o->building_friction,
                                (uint64_t)building->source_id)
          : NULL;
  city_part_t* walkway =
      roof ? city_chunk_add_part(chunk, "paving", PART_COLLIDER_MESH,
                                 COLLISION_LAYER_GROUND, o->pavement_friction,
                                 (uint64_t)building->source_id)
           : NULL;
  city_part_t* shops =
      walkway
          ? city_chunk_add_part(chunk, "shopfront", PART_COLLIDER_MESH,
                                COLLISION_LAYER_BUILDING, o->building_friction,
                                (uint64_t)building->source_id)
          : NULL;
  if (!walls || !roof || !walkway || !shops) return false;
  /* Adding parts may have moved the array: walls, roof, walkway and shops are
   * the last four. */
  walls = &chunk->parts[chunk->part_count - 4];
  roof = &chunk->parts[chunk->part_count - 3];
  walkway = &chunk->parts[chunk->part_count - 2];
  shops = &chunk->parts[chunk->part_count - 1];
  bool* arcaded = calloc(count, sizeof(bool));
  bool* shop = calloc(count, sizeof(bool));
  if (!arcaded || !shop) {
    free(arcaded);
    free(shop);
    return false;
  }
  double depth = building_depth(building);
  for (size_t i = 0; i < count; i++) {
    arcaded[i] = edge_faces_arcade(layout->model, o, building, i, depth);
    shop[i] = landmark == LANDMARK_NONE &&
              edge_faces_shops(layout->model, building, i);
  }
  float h = building->height;
  ridge_t ridge_data;
  const ridge_t* ridge = NULL;
  if (roof_shape(building) == ROOF_RIDGE) {
    bounding_rectangle_t r = bounding_rectangle(building->points, count);
    double half = (r.across_max - r.across_min) * 0.5;
    ridge_data.across = r.across;
    ridge_data.mid = (r.across_min + r.across_max) * 0.5;
    ridge_data.rise = fmin(fmax(half * 0.55, 0.8), 4.0);
    ridge_data.slope = ridge_data.rise / fmax(half, 0.5);
    ridge = &ridge_data;
  }
  /* On sloping ground the walls reach below the walkway so no gap shows. */
  float skirt = layout->terrain && layout->terrain->cols > 0 ? 3.0f : 0.0f;
  bool walls_ok = add_walls(layout, walls, walkway, shops, building->points,
                            count, h, ridge, skirt, arcaded, shop);
  free(arcaded);
  free(shop);
  if (!walls_ok) return false;
  for (size_t hole = 0; hole < building->hole_count; hole++)
    if (!add_walls(layout, walls, walkway, shops, building->holes[hole].points,
                   building->holes[hole].count, h, ridge, skirt, NULL, NULL))
      return false;
  bool pitched = false;
  if (!add_pitched_roof(roof, walls, building, h, o, &pitched)) return false;
  if (pitched) {
    if (!finish_landmark_building(layout, building, chunk, walls, roof,
                                  landmark, skirt))
      return false;
    stats->buildings++;
    return true;
  }
  size_t merged_capacity = count + 2 * building->hole_count;
  for (size_t hole = 0; hole < building->hole_count; hole++)
    merged_capacity += building->holes[hole].count;
  local_point_t* merged = malloc(merged_capacity * sizeof(local_point_t));
  uint32_t* indices = malloc(3 * merged_capacity * sizeof(uint32_t));
  const local_point_t** hole_points = calloc(
      building->hole_count ? building->hole_count : 1, sizeof(local_point_t*));
  size_t* hole_counts =
      calloc(building->hole_count ? building->hole_count : 1, sizeof(size_t));
  if (!merged || !indices || !hole_points || !hole_counts) {
    free(merged);
    free(indices);
    free(hole_points);
    free(hole_counts);
    return false;
  }
  for (size_t hole = 0; hole < building->hole_count; hole++) {
    hole_points[hole] = building->holes[hole].points;
    hole_counts[hole] = building->holes[hole].count;
  }
  size_t merged_count;
  bool clean;
  size_t triangles = triangulate_with_holes(
      building->points, count, hole_points, hole_counts, building->hole_count,
      merged, &merged_count, indices, &clean);
  if (!clean) {
    LOG_WARN("generate",
             "building %lld has an irregular footprint; roof approximated",
             (long long)building->source_id);
    stats->buildings_unclean++;
  }
  bool roof_ok = true;
  if (ridge) {
    for (size_t t = 0; t < triangles && roof_ok; t++) {
      local_point_t tri[3] = {merged[indices[t * 3]],
                              merged[indices[t * 3 + 1]],
                              merged[indices[t * 3 + 2]]};
      if (fabs(polygon_signed_area(tri, 3)) < 1e-6) continue;
      roof_ok = add_ridge_triangle(roof, tri, h, ridge);
    }
  } else {
    for (size_t i = 0; i < merged_count && roof_ok; i++)
      roof_ok =
          add_vertex(&roof->mesh, at(merged[i], h), vec3(0, 1, 0),
                     (float)(merged[i].x * 0.4), (float)(merged[i].z * 0.4));
    for (size_t t = 0; t < triangles && roof_ok; t++) {
      local_point_t tri[3] = {merged[indices[t * 3]],
                              merged[indices[t * 3 + 1]],
                              merged[indices[t * 3 + 2]]};
      if (fabs(polygon_signed_area(tri, 3)) < 1e-6) continue;
      roof_ok = add_face(&roof->mesh, indices[t * 3], indices[t * 3 + 1],
                         indices[t * 3 + 2], vec3(0, 1, 0));
    }
  }
  free(merged);
  free(indices);
  free(hole_points);
  free(hole_counts);
  if (!roof_ok) return false;
  if (!finish_landmark_building(layout, building, chunk, walls, roof, landmark,
                                skirt))
    return false;
  stats->buildings++;
  return true;
}

static bool generate_ground(layout_t* layout, city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  bool hilly = layout->terrain && layout->terrain->cols > 0;
  /* Flat ground is one quad per chunk; hills a grid of about 12 m cells,
   * a little below the roads laid on it. */
  int cells = hilly ? (int)ceil(o->chunk_size / 12.0f) : 1;
  for (int row = 0; row < layout->rows; row++) {
    for (int column = 0; column < layout->columns; column++) {
      city_chunk_t* chunk = city_package_chunk(layout->package, column, row);
      city_part_t* part =
          city_chunk_add_part(chunk, "ground", PART_COLLIDER_MESH,
                              COLLISION_LAYER_GROUND, o->ground_friction, 0);
      if (!part) return false;
      float x0 = (float)(layout->min_x + column * o->chunk_size);
      float z0 = (float)(layout->min_z + row * o->chunk_size);
      float step = o->chunk_size / (float)cells;
      float s = o->ground_uv_scale;
      uint32_t base = (uint32_t)part->mesh.vertex_count;
      for (int j = 0; j <= cells; j++)
        for (int i = 0; i <= cells; i++) {
          float x = x0 + step * (float)i, z = z0 + step * (float)j;
          float y = hilly ? (float)terrain_at(layout, x, z) - 0.05f : 0.0f;
          if (!add_vertex(&part->mesh, vec3(x, y, z), vec3(0, 1, 0), x * s,
                          z * s))
            return false;
        }
      for (int j = 0; j < cells; j++)
        for (int i = 0; i < cells; i++) {
          uint32_t a = base + (uint32_t)(j * (cells + 1) + i);
          uint32_t b = a + 1, d = a + (uint32_t)(cells + 1), c = d + 1;
          if (!add_face(&part->mesh, a, b, c, vec3(0, 1, 0)) ||
              !add_face(&part->mesh, a, c, d, vec3(0, 1, 0)))
            return false;
        }
      finish_part(chunk, part);
      stats->ground_tiles++;
    }
  }
  return true;
}

/* Lays everything but the ground and the buildings (already placed) on
 * the terrain: large surfaces are cut into small triangles first so they
 * follow the hills, then every vertex rises by the ground height under
 * it. Chunk bounds are rebuilt. */
static bool apply_terrain(layout_t* layout) {
  if (!layout->terrain || layout->terrain->cols == 0) return true;
  static const char* surfaces[] = {"park",     "paving", "parking", "junction",
                                   "pavement", "water",  "road"};
  for (size_t c = 0; c < layout->package->chunk_count; c++) {
    city_chunk_t* chunk = &layout->package->chunks[c];
    chunk->bounds = aabb_empty();
    for (size_t p = 0; p < chunk->part_count; p++) {
      city_part_t* part = &chunk->parts[p];
      bool skip = strcmp(part->material, "ground") == 0 ||
                  strncmp(part->material, "facade", 6) == 0 ||
                  strncmp(part->material, "roof", 4) == 0;
      if (!skip) {
        for (size_t k = 0; k < sizeof(surfaces) / sizeof(surfaces[0]); k++)
          if (strcmp(part->material, surfaces[k]) == 0 &&
              !mesh_data_subdivide(&part->mesh, 12.0f))
            return false;
        for (size_t v = 0; v < part->mesh.vertex_count; v++) {
          vec3_t* position = &part->mesh.vertices[v].position;
          position->y += (float)terrain_at(layout, position->x, position->z);
        }
      }
      chunk->bounds = aabb_merge(chunk->bounds, mesh_data_bounds(&part->mesh));
    }
  }
  return true;
}

/* The spawn: the longest ordinary road whose midpoint lies within the
 * central part of the extract, so a drive starts in town and the first
 * junction is close. */
static void choose_spawn(city_package_t* package, const city_model_t* model,
                         const city_generator_options_t* o) {
  const city_road_t* best = NULL;
  double best_length = 0;
  double extent = 0;
  for (size_t r = 0; r < model->road_count; r++)
    for (size_t i = 0; i < model->roads[r].point_count; i++) {
      local_point_t p = model->roads[r].points[i];
      extent = fmax(extent, fmax(fabs(p.x), fabs(p.z)));
    }
  for (int pass = 0; pass < 2 && !best; pass++) {
    double limit = pass == 0 ? extent * 0.4 : 1e30;
    for (size_t r = 0; r < model->road_count; r++) {
      const city_road_t* road = &model->roads[r];
      if (road->class == ROAD_CLASS_SERVICE ||
          road->class == ROAD_CLASS_MOTORWAY ||
          road->class == ROAD_CLASS_PEDESTRIAN ||
          road->class == ROAD_CLASS_PATH || road->class == ROAD_CLASS_WATERWAY)
        continue;
      local_point_t middle = centroid(road->points, road->point_count);
      if (fabs(middle.x) > limit || fabs(middle.z) > limit) continue;
      double length = 0;
      for (size_t i = 1; i < road->point_count; i++)
        length += v2_length(v2_sub(road->points[i], road->points[i - 1]));
      if (length > best_length) {
        best_length = length;
        best = road;
      }
    }
  }
  if (!best) best = &model->roads[0];
  /* Midpoint of the longest segment of the chosen road, facing the end of
   * the road nearer the centre of town so the drive heads into it. */
  size_t longest = 0;
  double longest_length = -1;
  for (size_t i = 1; i < best->point_count; i++) {
    double length = v2_length(v2_sub(best->points[i], best->points[i - 1]));
    if (length > longest_length) {
      longest_length = length;
      longest = i;
    }
  }
  local_point_t a = best->points[longest - 1], b = best->points[longest];
  local_point_t first = best->points[0],
                last = best->points[best->point_count - 1];
  bool towards_last = hypot(last.x, last.z) <= hypot(first.x, first.z);
  vec2_t direction =
      towards_last ? v2_normalize(v2_sub(b, a)) : v2_normalize(v2_sub(a, b));
  package->spawn_position =
      vec3((float)((a.x + b.x) * 0.5), o->road_height + 0.9f,
           (float)((a.z + b.z) * 0.5));
  package->spawn_yaw = (float)atan2(-direction.x, -direction.z);
}

/* The map's street names: each distinct name once, at the middle of its
 * longest piece, longest streets first. Capitals and plain ASCII only,
 * as the built-in font has nothing else. */
static void choose_labels(city_package_t* package, const city_model_t* model) {
  typedef struct {
    char text[32];
    double total, best;
    local_point_t at;
  } entry_t;
  entry_t* entries = calloc(model->road_count + 1, sizeof(entry_t));
  if (!entries) return;
  size_t count = 0;
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    if (!road->name[0] || road->class == ROAD_CLASS_WATERWAY ||
        road->class == ROAD_CLASS_PATH)
      continue;
    char text[32];
    size_t n = 0;
    for (const char* c = road->name; *c && n + 1 < sizeof(text); c++) {
      unsigned char u = (unsigned char)*c;
      if (u >= 0x80) continue;
      if (*c == '(') break; /* "(già Riviera...)" asides do not fit */
      text[n++] = *c == '_' ? ' ' : (char)toupper(u);
    }
    while (n > 0 && text[n - 1] == ' ') n--;
    text[n] = 0;
    if (n == 0) continue;
    double length = polyline_length(road->points, road->point_count);
    size_t e = 0;
    while (e < count && strcmp(entries[e].text, text) != 0) e++;
    if (e == count) {
      memcpy(entries[e].text, text, sizeof(text));
      count++;
    }
    entries[e].total += length;
    if (length > entries[e].best) {
      size_t segment;
      entries[e].best = length;
      entries[e].at = along_polyline(road->points, road->point_count,
                                     length * 0.5, &segment);
    }
  }
  for (size_t l = 0; l < CITY_PACKAGE_MAX_LABELS && l < count; l++) {
    size_t top = l;
    for (size_t e = l + 1; e < count; e++)
      if (entries[e].total > entries[top].total) top = e;
    entry_t chosen = entries[top];
    entries[top] = entries[l];
    entries[l] = chosen;
    city_label_t* label = &package->labels[package->label_count++];
    label->x = (float)chosen.at.x;
    label->z = (float)chosen.at.z;
    memcpy(label->text, chosen.text, sizeof(label->text));
  }
  free(entries);
}

/* The road as the runtime sees it: its centre line, and the places along
 * it where cars stand at the kerb. */
static bool add_route_and_parking(city_package_t* package,
                                  const city_model_t* model,
                                  const city_road_t* road, road_fit_t fit,
                                  road_cuts_t cuts) {
  if (road->class == ROAD_CLASS_WATERWAY) return true;
  bool drivable =
      road->class != ROAD_CLASS_PEDESTRIAN && road->class != ROAD_CLASS_PATH;
  city_route_t route;
  memset(&route, 0, sizeof(route));
  route.drivable = drivable;
  route.oneway = road->oneway;
  route.width = (float)(fit.half_width * 2);
  route.pavement = (float)fit.pavement;
  route.start_node = road->node_ids[0];
  route.end_node = road->node_ids[road->point_count - 1];
  /* Junction ids of zero come from rounding arcs; ends always carry real ones,
   * but keep the field non-zero for readers. */
  if (route.start_node == 0) route.start_node = 1;
  if (route.end_node == 0) route.end_node = 1;
  route.point_count = road->point_count;
  route.points = malloc(road->point_count * 2 * sizeof(float));
  if (!route.points) return false;
  for (size_t i = 0; i < road->point_count; i++) {
    route.points[2 * i] = (float)road->points[i].x;
    route.points[2 * i + 1] = (float)road->points[i].z;
  }
  if (!city_package_add_route(package, &route)) {
    free(route.points);
    return false;
  }
  (void)model;
  if (!drivable || road->class == ROAD_CLASS_MOTORWAY ||
      road->class == ROAD_CLASS_SERVICE || fit.half_width < 2.6 ||
      fit.pavement <= 0)
    return true;
  local_point_t* points = malloc(road->point_count * sizeof(local_point_t));
  vec2_t* laterals = malloc(road->point_count * sizeof(vec2_t));
  if (!points || !laterals) {
    free(points);
    free(laterals);
    return false;
  }
  size_t count = polyline_cut(road->points, road->point_count, cuts.start,
                              cuts.end, points);
  lateral_offsets(points, count, laterals);
  double length = polyline_length(points, count);
  uint64_t seed = (uint64_t)road->source_id * 0x9e3779b97f4a7c15ull + 7;
  bool ok = true;
  for (double along = 8.0; ok && along < length - 8.0; along += 6.2) {
    seed ^= seed >> 12, seed ^= seed << 25, seed ^= seed >> 27;
    if ((seed >> 20) % 100 >= 55) continue;
    size_t segment;
    local_point_t on_line = along_polyline(points, count, along, &segment);
    vec2_t lateral = v2_normalize(laterals[segment > 0 ? segment - 1 : 0]);
    int side = ((seed >> 33) & 1) ? 1 : -1;
    local_point_t spot =
        offset(on_line, lateral, side * (fit.half_width - 1.0));
    vec2_t direction = v2(-lateral.z, lateral.x); /* the way the road runs */
    direction = v2_normalize(direction);
    if (side < 0) direction = v2(-direction.x, -direction.z);
    double jitter = (double)((seed >> 41) % 100) / 100.0 * 0.06 - 0.03;
    city_parking_t place = {(float)spot.x, (float)spot.z,
                            (float)(atan2(-direction.x, -direction.z) + jitter),
                            (int)((seed >> 50) % 64)};
    ok = city_package_add_parking(package, place);
  }
  free(points);
  free(laterals);
  return ok;
}

bool city_generate(city_package_t* package, const city_model_t* model,
                   const city_generator_options_t* options, const char* name,
                   city_generator_stats_t* stats) {
  memset(stats, 0, sizeof(*stats));
  *package = city_package_create();
  if (model->road_count == 0) {
    LOG_ERROR("generate", "the model has no roads");
    return false;
  }
  snprintf(package->name, sizeof(package->name), "%s", name);
  snprintf(package->source, sizeof(package->source), "%s", model->source);
  snprintf(package->source_date, sizeof(package->source_date), "%s",
           model->import_date);
  time_t now = time(NULL);
  strftime(package->generated, sizeof(package->generated), "%Y-%m-%d",
           gmtime(&now));
  package->bounds_lat_lon[0] = model->bounds.min_lat;
  package->bounds_lat_lon[1] = model->bounds.min_lon;
  package->bounds_lat_lon[2] = model->bounds.max_lat;
  package->bounds_lat_lon[3] = model->bounds.max_lon;
  package->origin_lat = model->projection.origin_lat;
  package->origin_lon = model->projection.origin_lon;
  package->chunk_size = options->chunk_size;
  package->config_hash = city_generator_options_hash(options);

  /* Chunk grid over everything, with a margin so the ground extends past
   * the last road. */
  double min_x = 1e30, min_z = 1e30, max_x = -1e30, max_z = -1e30;
  for (size_t r = 0; r < model->road_count; r++)
    for (size_t i = 0; i < model->roads[r].point_count; i++) {
      local_point_t p = model->roads[r].points[i];
      min_x = fmin(min_x, p.x), max_x = fmax(max_x, p.x),
      min_z = fmin(min_z, p.z), max_z = fmax(max_z, p.z);
    }
  for (size_t b = 0; b < model->building_count; b++)
    for (size_t i = 0; i < model->buildings[b].point_count; i++) {
      local_point_t p = model->buildings[b].points[i];
      min_x = fmin(min_x, p.x), max_x = fmax(max_x, p.x),
      min_z = fmin(min_z, p.z), max_z = fmax(max_z, p.z);
    }
  /* Parks and water reach beyond the extract's edge: the ground must go
   * as far as they do, or they hang over nothing. */
  for (size_t a = 0; a < model->area_count; a++)
    for (size_t i = 0; i < model->areas[a].point_count; i++) {
      local_point_t p = model->areas[a].points[i];
      min_x = fmin(min_x, p.x), max_x = fmax(max_x, p.x),
      min_z = fmin(min_z, p.z), max_z = fmax(max_z, p.z);
    }
  const double margin = 40.0;
  layout_t layout;
  layout.package = package;
  layout.options = options;
  layout.terrain = &model->terrain;
  layout.model = model;
  layout.arcade_edges = NULL;
  layout.arcade_edge_count = 0;
  layout.min_x =
      floor((min_x - margin) / options->chunk_size) * options->chunk_size;
  layout.min_z =
      floor((min_z - margin) / options->chunk_size) * options->chunk_size;
  layout.columns =
      (int)ceil((max_x + margin - layout.min_x) / options->chunk_size);
  layout.rows =
      (int)ceil((max_z + margin - layout.min_z) / options->chunk_size);
  if (layout.columns < 1) layout.columns = 1;
  if (layout.rows < 1) layout.rows = 1;

  road_fit_t* fits = calloc(model->road_count, sizeof(road_fit_t));
  road_cuts_t* cuts = calloc(model->road_count, sizeof(road_cuts_t));
  if (!fits || !cuts) {
    free(fits);
    free(cuts);
    return false;
  }
  for (size_t r = 0; r < model->road_count; r++)
    fits[r] = fit_road(model, &model->roads[r], options);
  bool ok = generate_ground(&layout, stats);
  for (size_t j = 0; ok && j < model->junction_count; j++)
    ok = generate_junction(&layout, model, &model->junctions[j], fits, cuts,
                           stats);
  for (size_t r = 0; ok && r < model->road_count; r++)
    ok = generate_road(&layout, model, &model->roads[r], fits[r], cuts[r],
                       stats);
  for (size_t r = 0; ok && r < model->road_count; r++)
    ok = add_route_and_parking(package, model, &model->roads[r], fits[r],
                               cuts[r]);
  for (size_t a = 0; ok && a < model->area_count; a++)
    ok = generate_area(&layout, &model->areas[a], stats);
  /* Edges that face an arcaded street, so neighbouring porticoes join. */
  if (options->arcade_streets[0]) {
    layout.arcade_edges =
        calloc(model->building_count * 8 + 8, sizeof(arcade_edge_t));
    size_t capacity = model->building_count * 8 + 8;
    for (size_t b = 0; ok && b < model->building_count; b++) {
      const city_building_t* building = &model->buildings[b];
      double depth = building_depth(building);
      for (size_t i = 0; i < building->point_count; i++)
        if (edge_faces_arcade(model, options, building, i, depth) &&
            layout.arcade_edge_count < capacity) {
          arcade_edge_t* edge =
              &layout.arcade_edges[layout.arcade_edge_count++];
          edge->a = building->points[i];
          edge->b = building->points[(i + 1) % building->point_count];
        }
    }
  }
  for (size_t b = 0; ok && b < model->building_count; b++)
    ok = generate_building(&layout, &model->buildings[b], stats);
  if (ok) ok = generate_street_trees(&layout, model, stats);
  if (ok) ok = generate_furniture(&layout, model, stats);
  free(layout.arcade_edges);
  free(fits);
  free(cuts);
  ok = ok && apply_terrain(&layout);
  if (!ok) {
    LOG_ERROR("generate", "out of memory while generating");
    return false;
  }
  choose_spawn(package, model, options);
  choose_labels(package, model);
  for (size_t f = 0; f < model->furniture_count; f++)
    if (model->furniture[f].kind == OSM_FURNITURE_SIGNALS)
      city_package_add_signal(package, (float)model->furniture[f].x,
                              (float)model->furniture[f].z);
  /* The car starts on an empty road. */
  size_t kept = 0;
  for (size_t i = 0; i < package->parked_count; i++) {
    float dx = package->parked[i].x - package->spawn_position.x;
    float dz = package->parked[i].z - package->spawn_position.z;
    if (dx * dx + dz * dz > 100.0f)
      package->parked[kept++] = package->parked[i];
  }
  package->parked_count = kept;
  package->spawn_position.y += (float)terrain_at(
      &layout, package->spawn_position.x, package->spawn_position.z);
  LOG_INFO("generate",
           "%d x %d chunks, %zu roads, %zu junction patches, %zu pavements, "
           "%zu lamps, %zu areas, %zu trees, %zu buildings (%zu approximated), "
           "%zu triangles",
           layout.columns, layout.rows, stats->roads, stats->junction_patches,
           stats->pavements, stats->lamps, stats->areas, stats->trees,
           stats->buildings, stats->buildings_unclean,
           city_package_triangle_count(package));
  return true;
}
