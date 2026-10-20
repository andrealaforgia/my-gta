#include "city_generator.h"

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
                                3.0f,   3.2f,  4,     0.25f, 1.0f,  0.9f,
                                0.7f,   0.6f,  28.0f, 14.0f};
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
           "%g %g %d %g %g %g %g %g %d %g %g %g %g %g %g %g", o->chunk_size,
           o->road_height, o->pavements, o->pavement_width, o->kerb_height,
           o->road_texture_length, o->window_width, o->storey_height,
           o->facade_variants, o->ground_uv_scale, o->road_friction,
           o->pavement_friction, o->ground_friction, o->building_friction,
           o->lamp_spacing, o->tree_spacing);
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
                      const vec2_t* laterals, size_t count, double from,
                      double to, float y, float texture_length) {
  uint32_t base = (uint32_t)mesh->vertex_count;
  double distance = 0;
  for (size_t i = 0; i < count; i++) {
    if (i > 0) distance += v2_length(v2_sub(points[i], points[i - 1]));
    float v = (float)(distance / texture_length);
    if (!add_vertex(mesh, at(offset(points[i], laterals[i], from), y),
                    vec3(0, 1, 0), 0, v))
      return false;
    if (!add_vertex(mesh, at(offset(points[i], laterals[i], to), y),
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
                     const vec2_t* laterals, size_t count, double along,
                     float y0, float y1, bool facing_right) {
  uint32_t base = (uint32_t)mesh->vertex_count;
  for (size_t i = 0; i < count; i++) {
    vec2_t right = v2_normalize(laterals[i]);
    vec3_t normal = vec3((float)(facing_right ? right.x : -right.x), 0,
                         (float)(facing_right ? right.z : -right.z));
    local_point_t p = offset(points[i], laterals[i], along);
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

/* The nearest building edge to the road's centre line, sampled every two
 * metres, or a large number when nothing is near. */
static double corridor_half_width(const city_model_t* model,
                                  const city_road_t* road, double reach) {
  double min_x = 1e30, max_x = -1e30, min_z = 1e30, max_z = -1e30;
  for (size_t i = 0; i < road->point_count; i++) {
    min_x = fmin(min_x, road->points[i].x),
    max_x = fmax(max_x, road->points[i].x);
    min_z = fmin(min_z, road->points[i].z),
    max_z = fmax(max_z, road->points[i].z);
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
                                const city_road_t* road, double reach) {
  double nearest = 1e30;
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* other = &model->roads[r];
    if (other == road || other->class == ROAD_CLASS_PATH ||
        other->class == ROAD_CLASS_WATERWAY)
      continue;
    bool shares_node = false;
    for (size_t i = 0; i < road->point_count && !shares_node; i++)
      for (size_t k = 0; k < other->point_count && !shares_node; k++)
        if (road->node_ids[i] == other->node_ids[k]) shares_node = true;
    if (shares_node) continue;
    for (size_t i = 1; i < road->point_count; i++) {
      local_point_t p = road->points[i - 1], q = road->points[i];
      vec2_t direction = v2_normalize(v2_sub(q, p));
      double length = hypot(q.x - p.x, q.z - p.z);
      int samples = (int)(length / 3.0) + 1;
      for (int k = 0; k <= samples; k++) {
        double t = (double)k / (double)samples;
        local_point_t sample = {p.x + (q.x - p.x) * t, p.z + (q.z - p.z) * t};
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

static road_fit_t fit_road(const city_model_t* model, const city_road_t* road,
                           const city_generator_options_t* o) {
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
      model, road, 2.0 * (fit.half_width + fit.pavement + 1.0));
  if (to_roads < fit.half_width + fit.pavement + gap) {
    double available = fmax(to_roads - 0.1, narrowest);
    fit.half_width = fmin(fit.half_width, available);
    fit.pavement = fmin(fmax(available - fit.half_width, 0.0), fit.pavement);
    if (fit.pavement < 0.4) fit.pavement = 0;
  }
  double wanted = fit.half_width + fit.pavement + gap;
  double corridor = corridor_half_width(model, road, wanted + 1.0);
  if (corridor >= wanted) return fit;
  double available = fmax(corridor - gap, narrowest);
  fit.half_width =
      fmin(fit.half_width, fmax(narrowest, available - fit.pavement));
  fit.pavement = fmin(fmax(available - fit.half_width, 0.0), fit.pavement);
  if (fit.pavement < 0.4) fit.pavement = 0;
  return fit;
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
  if (shape.corner_count == 0) return true;
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
  return true;
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
                           size_t count, road_fit_t fit,
                           city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  double length = polyline_length(points, count);
  if (length < o->lamp_spacing * 0.6) return true;
  city_chunk_t* chunk = chunk_at(layout, centroid(points, count));
  city_part_t* part = city_chunk_add_part(chunk, "lamp", PART_COLLIDER_MESH,
                                          COLLISION_LAYER_PROP, 0.5f,
                                          (uint64_t)road->source_id);
  if (!part) return false;
  const float height = 4.5f;
  int side = 1;
  for (double along = o->lamp_spacing * 0.5; along < length;
       along += o->lamp_spacing, side = -side) {
    size_t segment;
    local_point_t on_line = along_polyline(points, count, along, &segment);
    vec2_t lateral = v2_normalize(laterals[segment > 0 ? segment - 1 : 0]);
    double post_offset = side * (fit.half_width + fit.pavement * 0.5);
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
            &part->mesh,
            vec3((float)head.x, y0 + height - 0.12f, (float)head.z),
            vec3(0.22f, 0.1f, 0.22f)))
      return false;
    stats->lamps++;
  }
  if (part->mesh.vertex_count == 0) {
    mesh_data_free(&part->mesh);
    chunk->part_count--;
    return true;
  }
  finish_part(chunk, part);
  return true;
}

/* ---- roads ---- */

static bool generate_road(layout_t* layout, const city_road_t* road,
                          road_fit_t fit, road_cuts_t cuts,
                          city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
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
  city_chunk_t* chunk = chunk_at(layout, centroid(points, count));
  bool for_people =
      road->class == ROAD_CLASS_PEDESTRIAN || road->class == ROAD_CLASS_PATH;
  bool water = road->class == ROAD_CLASS_WATERWAY;
  /* Paths sit a little above the roads they cross; streams just above the
   * grass and below everything else, with nothing to collide with. */
  float height = road->class == ROAD_CLASS_PATH ? o->road_height + 0.012f
                 : water                        ? 0.02f
                                                : o->road_height;
  city_part_t* part = city_chunk_add_part(
      chunk,
      water        ? "water"
      : for_people ? "paving"
                   : "road",
      water ? PART_COLLIDER_NONE : PART_COLLIDER_MESH, COLLISION_LAYER_GROUND,
      o->road_friction, (uint64_t)road->source_id);
  double half = fit.half_width;
  bool ok =
      part && add_strip(&part->mesh, points, laterals, count, -half, half,
                        height, for_people ? 3.0f : o->road_texture_length);
  if (ok) finish_part(chunk, part);
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
      double inner = side * half, outer = side * (half + fit.pavement);
      ok = add_strip(&pavement->mesh, points, laterals, count,
                     fmin(inner, outer), fmax(inner, outer), o->kerb_height,
                     o->pavement_width) &&
           add_kerb(&kerb->mesh, points, laterals, count, inner, o->road_height,
                    o->kerb_height, side < 0);
    }
    if (ok) {
      finish_part(chunk, pavement);
      finish_part(chunk, kerb);
      stats->pavements++;
    }
  }
  if (ok && o->lamp_spacing > 0 && road->class != ROAD_CLASS_PATH &&
      road->class != ROAD_CLASS_SERVICE && road->class != ROAD_CLASS_WATERWAY)
    ok = generate_lamps(layout, road, points, laterals, count, fit, stats);
  free(points);
  free(laterals);
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
      float height = 4.5f + (float)((seed >> 40) % 300) / 100.0f;
      float crown = 1.6f + (float)((seed >> 50) % 100) / 100.0f;
      if (!mesh_data_add_box(&trunks->mesh,
                             vec3((float)p.x, height * 0.5f, (float)p.z),
                             vec3(0.18f, height * 0.5f, 0.18f)))
        return false;
      vec3_t top = vec3((float)p.x, height, (float)p.z);
      if (!mesh_data_add_box(&leaves->mesh,
                             vec3_add(top, vec3(0, crown * 0.5f, 0)),
                             vec3(crown * 0.8f, crown * 0.9f, crown * 0.8f)) ||
          !mesh_data_add_box(&leaves->mesh,
                             vec3_add(top, vec3(0, crown * 0.6f, 0)),
                             vec3(crown * 0.7f, crown * 0.6f, crown * 0.7f)) ||
          !mesh_data_add_box(
              &leaves->mesh,
              vec3_add(top, vec3(crown * 0.5f, -crown * 0.2f, crown * 0.3f)),
              vec3(crown * 0.6f, crown * 0.5f, crown * 0.6f)))
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
static bool add_walls(city_part_t* walls, const local_point_t* ring,
                      size_t count, float h,
                      const city_generator_options_t* o) {
  for (size_t i = 0; i < count; i++) {
    local_point_t a = ring[i], b = ring[(i + 1) % count];
    vec2_t along = v2_sub(b, a);
    double length = v2_length(along);
    if (length < 1e-6) continue;
    vec2_t direction = v2_normalize(along);
    vec3_t normal = vec3((float)-direction.z, 0, (float)direction.x);
    float u = (float)(length / o->window_width), v = h / o->storey_height;
    uint32_t base = (uint32_t)walls->mesh.vertex_count;
    if (!add_vertex(&walls->mesh, at(a, 0), normal, 0, 0) ||
        !add_vertex(&walls->mesh, at(b, 0), normal, u, 0) ||
        !add_vertex(&walls->mesh, at(b, h), normal, u, v) ||
        !add_vertex(&walls->mesh, at(a, h), normal, 0, v))
      return false;
    if (!add_face(&walls->mesh, base, base + 1, base + 2, normal) ||
        !add_face(&walls->mesh, base, base + 2, base + 3, normal))
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

/* A hipped roof with eaves on footprints that nearly fill their bounding
 * rectangle and have no courtyard. Sets *done when it built one. */
static bool add_hipped_roof(city_part_t* roof, const city_building_t* building,
                            float h, const city_generator_options_t* o,
                            bool* done) {
  *done = false;
  if (building->hole_count > 0 || building->point_count < 4) return true;
  bounding_rectangle_t r =
      bounding_rectangle(building->points, building->point_count);
  double footprint =
      fabs(polygon_signed_area(building->points, building->point_count));
  if (r.area <= 0 || footprint / r.area < 0.82) return true;
  const double eaves = 0.35;
  double a0 = r.along_min - eaves, a1 = r.along_max + eaves;
  double c0 = r.across_min - eaves, c1 = r.across_max + eaves;
  double half_span = (c1 - c0) * 0.5, mid = (c0 + c1) * 0.5;
  double inset = fmin(half_span, (a1 - a0) * 0.5);
  float rise = (float)fmin(half_span * 0.6, 4.0);
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
      !add_face(&roof->mesh, base + 2, base + 4, base + 5, up) ||
      !add_face(&roof->mesh, base + 1, base + 2, base + 5, up) ||
      !add_face(&roof->mesh, base + 3, base + 0, base + 4, up))
    return false;
  uint32_t under = (uint32_t)roof->mesh.vertex_count;
  vec3_t down = vec3(0, -1, 0);
  for (int i = 0; i < 4; i++)
    if (!add_vertex(&roof->mesh, corners[i], down, (float)(us[i] * s),
                    (float)(vs[i] * s)))
      return false;
  if (!add_face(&roof->mesh, under + 0, under + 1, under + 2, down) ||
      !add_face(&roof->mesh, under + 0, under + 2, under + 3, down))
    return false;
  (void)o;
  *done = true;
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
  city_part_t* walls = city_chunk_add_part(
      chunk, facade, PART_COLLIDER_MESH, COLLISION_LAYER_BUILDING,
      o->building_friction, (uint64_t)building->source_id);
  city_part_t* roof =
      walls ? city_chunk_add_part(
                  chunk, "roof", PART_COLLIDER_MESH, COLLISION_LAYER_BUILDING,
                  o->building_friction, (uint64_t)building->source_id)
            : NULL;
  if (!walls || !roof) return false;
  walls = &chunk->parts[chunk->part_count -
                        2]; /* the roof's addition may have moved the array */
  float h = building->height;
  if (!add_walls(walls, building->points, count, h, o)) return false;
  for (size_t hole = 0; hole < building->hole_count; hole++)
    if (!add_walls(walls, building->holes[hole].points,
                   building->holes[hole].count, h, o))
      return false;
  bool hipped = false;
  if (!add_hipped_roof(roof, building, h, o, &hipped)) return false;
  if (hipped) {
    finish_part(chunk, walls);
    finish_part(chunk, roof);
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
  for (size_t i = 0; i < merged_count && roof_ok; i++)
    roof_ok =
        add_vertex(&roof->mesh, at(merged[i], h), vec3(0, 1, 0),
                   (float)(merged[i].x * 0.4), (float)(merged[i].z * 0.4));
  for (size_t t = 0; t < triangles && roof_ok; t++) {
    local_point_t tri[3] = {merged[indices[t * 3]], merged[indices[t * 3 + 1]],
                            merged[indices[t * 3 + 2]]};
    if (fabs(polygon_signed_area(tri, 3)) < 1e-6) continue;
    roof_ok = add_face(&roof->mesh, indices[t * 3], indices[t * 3 + 1],
                       indices[t * 3 + 2], vec3(0, 1, 0));
  }
  free(merged);
  free(indices);
  free(hole_points);
  free(hole_counts);
  if (!roof_ok) return false;
  finish_part(chunk, walls);
  finish_part(chunk, roof);
  stats->buildings++;
  return true;
}

static bool generate_ground(layout_t* layout, city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  for (int row = 0; row < layout->rows; row++) {
    for (int column = 0; column < layout->columns; column++) {
      city_chunk_t* chunk = city_package_chunk(layout->package, column, row);
      city_part_t* part =
          city_chunk_add_part(chunk, "grass", PART_COLLIDER_MESH,
                              COLLISION_LAYER_GROUND, o->ground_friction, 0);
      if (!part) return false;
      float x0 = (float)(layout->min_x + column * o->chunk_size),
            x1 = x0 + o->chunk_size;
      float z0 = (float)(layout->min_z + row * o->chunk_size),
            z1 = z0 + o->chunk_size;
      float s = o->ground_uv_scale;
      uint32_t base = (uint32_t)part->mesh.vertex_count;
      if (!add_vertex(&part->mesh, vec3(x0, 0, z0), vec3(0, 1, 0), x0 * s,
                      z0 * s) ||
          !add_vertex(&part->mesh, vec3(x1, 0, z0), vec3(0, 1, 0), x1 * s,
                      z0 * s) ||
          !add_vertex(&part->mesh, vec3(x1, 0, z1), vec3(0, 1, 0), x1 * s,
                      z1 * s) ||
          !add_vertex(&part->mesh, vec3(x0, 0, z1), vec3(0, 1, 0), x0 * s,
                      z1 * s))
        return false;
      if (!add_face(&part->mesh, base, base + 1, base + 2, vec3(0, 1, 0)) ||
          !add_face(&part->mesh, base, base + 2, base + 3, vec3(0, 1, 0)))
        return false;
      finish_part(chunk, part);
      stats->ground_tiles++;
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
  const double margin = 40.0;
  layout_t layout;
  layout.package = package;
  layout.options = options;
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
    ok = generate_road(&layout, &model->roads[r], fits[r], cuts[r], stats);
  for (size_t a = 0; ok && a < model->area_count; a++)
    ok = generate_area(&layout, &model->areas[a], stats);
  for (size_t b = 0; ok && b < model->building_count; b++)
    ok = generate_building(&layout, &model->buildings[b], stats);
  free(fits);
  free(cuts);
  if (!ok) {
    LOG_ERROR("generate", "out of memory while generating");
    return false;
  }
  choose_spawn(package, model, options);
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
