#include "city_generator.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "config.h"
#include "log.h"
#include "physics_world.h"
#include "triangulate.h"

#define PATCH_SEGMENTS 20

city_generator_options_t city_generator_default_options(void) {
  city_generator_options_t o = {150.0f, 0.02f, 1.15f, 0.25f, true, 2.0f, 0.12f, 8.0f, 3.0f, 3.2f, 4,
                                0.25f, 1.0f, 0.9f, 0.7f, 0.6f};
  return o;
}

bool city_generator_options_load(city_generator_options_t* o, const char* path) {
  *o = city_generator_default_options();
  config_t config;
  if (!config_load(&config, path)) {
    LOG_ERROR("generate", "cannot read %s", path);
    config_free(&config);
    return false;
  }
  o->chunk_size = config_get_float(&config, "chunk_size", o->chunk_size);
  o->road_height = config_get_float(&config, "road_height", o->road_height);
  o->junction_radius_scale = config_get_float(&config, "junction_radius_scale", o->junction_radius_scale);
  o->bend_patch_angle = config_get_float(&config, "bend_patch_angle", o->bend_patch_angle);
  o->pavements = config_get_bool(&config, "pavements", o->pavements);
  o->pavement_width = config_get_float(&config, "pavement_width", o->pavement_width);
  o->kerb_height = config_get_float(&config, "kerb_height", o->kerb_height);
  o->road_texture_length = config_get_float(&config, "road_texture_length", o->road_texture_length);
  o->window_width = config_get_float(&config, "window_width", o->window_width);
  o->storey_height = config_get_float(&config, "storey_height", o->storey_height);
  o->facade_variants = config_get_int(&config, "facade_variants", o->facade_variants);
  o->ground_uv_scale = config_get_float(&config, "ground_uv_scale", o->ground_uv_scale);
  o->road_friction = config_get_float(&config, "road_friction", o->road_friction);
  o->pavement_friction = config_get_float(&config, "pavement_friction", o->pavement_friction);
  o->ground_friction = config_get_float(&config, "ground_friction", o->ground_friction);
  o->building_friction = config_get_float(&config, "building_friction", o->building_friction);
  if (config_defaulted_count(&config) > 0)
    LOG_WARN("generate", "%s: %zu settings missing, defaults used", path, config_defaulted_count(&config));
  config_free(&config);
  bool valid = o->chunk_size >= 20 && o->facade_variants >= 1;
  if (!valid) LOG_ERROR("generate", "%s: chunk_size must be >= 20 and facade_variants >= 1", path);
  return valid;
}

uint64_t city_generator_options_hash(const city_generator_options_t* o) {
  /* FNV-1a over a textual rendering so padding bytes do not matter. */
  char text[512];
  snprintf(text, sizeof(text), "%g %g %g %g %d %g %g %g %g %g %d %g %g %g %g %g", o->chunk_size,
           o->road_height, o->junction_radius_scale, o->bend_patch_angle, o->pavements, o->pavement_width,
           o->kerb_height, o->road_texture_length, o->window_width, o->storey_height, o->facade_variants,
           o->ground_uv_scale, o->road_friction, o->pavement_friction, o->ground_friction,
           o->building_friction);
  uint64_t hash = 1469598103934665603ull;
  for (const char* c = text; *c; c++) hash = (hash ^ (uint8_t)*c) * 1099511628211ull;
  return hash;
}

/* ---- mesh helpers ---- */

static vec3_t at(local_point_t p, float y) { return vec3((float)p.x, y, (float)p.z); }

static bool add_vertex(mesh_data_t* mesh, vec3_t position, vec3_t normal, float u, float v) {
  vertex_t vertex = {position, normal, u, v};
  return mesh_data_add_vertex(mesh, vertex);
}

/* Adds a, b, c wound so the face normal points along `up_hint`. */
static bool add_face(mesh_data_t* mesh, uint32_t a, uint32_t b, uint32_t c, vec3_t up_hint) {
  vec3_t pa = mesh->vertices[a].position, pb = mesh->vertices[b].position, pc = mesh->vertices[c].position;
  vec3_t normal = vec3_cross(vec3_sub(pb, pa), vec3_sub(pc, pa));
  if (vec3_dot(normal, up_hint) < 0) return mesh_data_add_triangle(mesh, a, c, b);
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
static vec2_t v2_sub(local_point_t a, local_point_t b) { return v2(a.x - b.x, a.z - b.z); }
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

/* ---- junction radii and road trimming ---- */

typedef struct {
  int64_t node_id;
  float radius;
  float max_turn; /* radians between the two roads at a degree-2 node */
  bool patch;
} junction_info_t;

static const junction_info_t* junction_for(const junction_info_t* infos, size_t count, int64_t node) {
  for (size_t i = 0; i < count; i++)
    if (infos[i].node_id == node) return &infos[i];
  return NULL;
}

static double heading_at_end(const city_road_t* road, bool start) {
  local_point_t a = start ? road->points[0] : road->points[road->point_count - 1];
  local_point_t b = start ? road->points[1] : road->points[road->point_count - 2];
  vec2_t away = v2_normalize(v2_sub(b, a)); /* direction leaving the junction */
  return atan2(away.z, away.x);
}

static junction_info_t* compute_junctions(const city_model_t* model, const city_generator_options_t* o,
                                          size_t* count) {
  junction_info_t* infos = calloc(model->junction_count ? model->junction_count : 1, sizeof(junction_info_t));
  if (!infos) return NULL;
  for (size_t j = 0; j < model->junction_count; j++) {
    const city_junction_t* junction = &model->junctions[j];
    junction_info_t* info = &infos[j];
    info->node_id = junction->node_id;
    double headings[2] = {0, 0};
    int ends = 0;
    for (size_t r = 0; r < model->road_count; r++) {
      const city_road_t* road = &model->roads[r];
      for (int end = 0; end < 2; end++) {
        int64_t node = road->node_ids[end ? road->point_count - 1 : 0];
        if (node != junction->node_id) continue;
        if (road->width * 0.5f > info->radius) info->radius = road->width * 0.5f;
        if (ends < 2) headings[ends] = heading_at_end(road, end == 0);
        ends++;
      }
    }
    info->radius *= o->junction_radius_scale;
    if (junction->degree == 2 && ends == 2) {
      double turn = fabs(fmod(fabs(headings[0] - headings[1]), 2 * M_PI));
      info->max_turn = (float)fabs(M_PI - turn); /* 0 when the roads continue straight */
    }
    info->patch = junction->degree >= 3 || (junction->degree == 2 && info->max_turn > o->bend_patch_angle);
  }
  *count = model->junction_count;
  return infos;
}

/* Copies the road's polyline with the ends pulled back from patched junctions. */
static size_t trimmed_polyline(const city_road_t* road, const junction_info_t* infos, size_t info_count,
                               float extra, local_point_t* out) {
  size_t count = road->point_count;
  memcpy(out, road->points, count * sizeof(local_point_t));
  for (int end = 0; end < 2; end++) {
    size_t index = end ? count - 1 : 0;
    size_t neighbour = end ? count - 2 : 1;
    const junction_info_t* info = junction_for(infos, info_count, road->node_ids[index]);
    if (!info || !info->patch) continue;
    vec2_t inward = v2_sub(out[neighbour], out[index]);
    double length = v2_length(inward);
    double trim = info->radius + extra;
    if (trim > length * 0.45) trim = length * 0.45;
    out[index] = offset(out[index], v2_normalize(inward), trim);
  }
  return count;
}

/* Lateral unit offsets per point with mitre at the interior joints. */
static void lateral_offsets(const local_point_t* points, size_t count, vec2_t* laterals) {
  for (size_t i = 0; i < count; i++) {
    vec2_t before = i > 0 ? v2_normalize(v2_sub(points[i], points[i - 1])) : v2(0, 0);
    vec2_t after = i + 1 < count ? v2_normalize(v2_sub(points[i + 1], points[i])) : v2(0, 0);
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
static bool add_strip(mesh_data_t* mesh, const local_point_t* points, const vec2_t* laterals, size_t count,
                      double from, double to, float y, float texture_length) {
  uint32_t base = (uint32_t)mesh->vertex_count;
  double distance = 0;
  for (size_t i = 0; i < count; i++) {
    if (i > 0) distance += v2_length(v2_sub(points[i], points[i - 1]));
    float v = (float)(distance / texture_length);
    if (!add_vertex(mesh, at(offset(points[i], laterals[i], from), y), vec3(0, 1, 0), 0, v)) return false;
    if (!add_vertex(mesh, at(offset(points[i], laterals[i], to), y), vec3(0, 1, 0), 1, v)) return false;
  }
  for (size_t i = 0; i + 1 < count; i++) {
    uint32_t a = base + (uint32_t)i * 2, b = a + 1, c = a + 2, d = a + 3;
    if (!add_face(mesh, a, b, d, vec3(0, 1, 0)) || !add_face(mesh, a, d, c, vec3(0, 1, 0))) return false;
  }
  return true;
}

/* A vertical face along the offset line, from y0 to y1, facing `outward`. */
static bool add_kerb(mesh_data_t* mesh, const local_point_t* points, const vec2_t* laterals, size_t count,
                     double along, float y0, float y1, bool facing_right) {
  uint32_t base = (uint32_t)mesh->vertex_count;
  for (size_t i = 0; i < count; i++) {
    vec2_t right = v2_normalize(laterals[i]);
    vec3_t normal = vec3((float)(facing_right ? right.x : -right.x), 0, (float)(facing_right ? right.z : -right.z));
    local_point_t p = offset(points[i], laterals[i], along);
    if (!add_vertex(mesh, at(p, y0), normal, (float)i, 0) || !add_vertex(mesh, at(p, y1), normal, (float)i, 1))
      return false;
  }
  for (size_t i = 0; i + 1 < count; i++) {
    uint32_t a = base + (uint32_t)i * 2, b = a + 1, c = a + 2, d = a + 3;
    vec3_t hint = mesh->vertices[a].normal;
    if (!add_face(mesh, a, b, d, hint) || !add_face(mesh, a, d, c, hint)) return false;
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

static bool generate_road(layout_t* layout, const city_road_t* road, const junction_info_t* infos,
                          size_t info_count, city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  size_t count = road->point_count;
  local_point_t* points = malloc(count * sizeof(local_point_t));
  vec2_t* laterals = malloc(count * sizeof(vec2_t));
  if (!points || !laterals) {
    free(points);
    free(laterals);
    return false;
  }
  trimmed_polyline(road, infos, info_count, 0.0f, points);
  lateral_offsets(points, count, laterals);
  city_chunk_t* chunk = chunk_at(layout, centroid(points, count));
  city_part_t* part = city_chunk_add_part(chunk, "road", PART_COLLIDER_MESH, COLLISION_LAYER_GROUND,
                                          o->road_friction, (uint64_t)road->source_id);
  double half = road->width * 0.5;
  bool ok = part && add_strip(&part->mesh, points, laterals, count, -half, half, o->road_height,
                              o->road_texture_length);
  if (ok) finish_part(chunk, part);
  stats->roads++;

  if (ok && o->pavements && road->class != ROAD_CLASS_SERVICE) {
    trimmed_polyline(road, infos, info_count, o->pavement_width, points);
    lateral_offsets(points, count, laterals);
    /* Adding a part may move the chunk's part array, so take pointers only
     * once both parts exist. */
    city_part_t* pavement = city_chunk_add_part(chunk, "pavement", PART_COLLIDER_MESH, COLLISION_LAYER_GROUND,
                                                o->pavement_friction, (uint64_t)road->source_id);
    city_part_t* kerb = pavement ? city_chunk_add_part(chunk, "kerb", PART_COLLIDER_NONE, 0, 0, (uint64_t)road->source_id)
                                 : NULL;
    ok = pavement && kerb;
    if (ok) pavement = &chunk->parts[chunk->part_count - 2];
    for (int side = -1; side <= 1 && ok; side += 2) {
      double inner = side * half, outer = side * (half + o->pavement_width);
      ok = add_strip(&pavement->mesh, points, laterals, count, fmin(inner, outer), fmax(inner, outer),
                     o->kerb_height, o->pavement_width) &&
           add_kerb(&kerb->mesh, points, laterals, count, inner, o->road_height, o->kerb_height, side < 0);
    }
    if (ok) {
      finish_part(chunk, pavement);
      finish_part(chunk, kerb);
      stats->pavements++;
    }
  }
  free(points);
  free(laterals);
  return ok;
}

static bool generate_patch(layout_t* layout, const city_junction_t* junction, const junction_info_t* info,
                           city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  city_chunk_t* chunk = chunk_at(layout, junction->position);
  city_part_t* part = city_chunk_add_part(chunk, "junction", PART_COLLIDER_MESH, COLLISION_LAYER_GROUND,
                                          o->road_friction, (uint64_t)junction->node_id);
  if (!part) return false;
  float y = o->road_height + 0.004f;
  uint32_t centre = (uint32_t)part->mesh.vertex_count;
  if (!add_vertex(&part->mesh, at(junction->position, y), vec3(0, 1, 0), 0.5f, 0.5f)) return false;
  for (int i = 0; i < PATCH_SEGMENTS; i++) {
    double angle = 2.0 * M_PI * i / PATCH_SEGMENTS;
    local_point_t p = offset(junction->position, v2(cos(angle), sin(angle)), info->radius);
    if (!add_vertex(&part->mesh, at(p, y), vec3(0, 1, 0), 0.5f + 0.5f * (float)cos(angle),
                    0.5f + 0.5f * (float)sin(angle)))
      return false;
  }
  for (int i = 0; i < PATCH_SEGMENTS; i++) {
    uint32_t a = centre + 1 + (uint32_t)i, b = centre + 1 + (uint32_t)((i + 1) % PATCH_SEGMENTS);
    if (!add_face(&part->mesh, centre, a, b, vec3(0, 1, 0))) return false;
  }
  finish_part(chunk, part);
  stats->junction_patches++;
  return true;
}

static bool generate_building(layout_t* layout, const city_building_t* building, city_generator_stats_t* stats) {
  const city_generator_options_t* o = layout->options;
  size_t count = building->point_count;
  city_chunk_t* chunk = chunk_at(layout, centroid(building->points, count));
  char facade[32];
  unsigned variant = (unsigned)((uint64_t)building->source_id * 2654435761u % (unsigned)o->facade_variants);
  snprintf(facade, sizeof(facade), "facade%u", variant);
  city_part_t* walls = city_chunk_add_part(chunk, facade, PART_COLLIDER_MESH, COLLISION_LAYER_BUILDING,
                                           o->building_friction, (uint64_t)building->source_id);
  city_part_t* roof = walls ? city_chunk_add_part(chunk, "roof", PART_COLLIDER_MESH, COLLISION_LAYER_BUILDING,
                                                  o->building_friction, (uint64_t)building->source_id)
                            : NULL;
  if (!walls || !roof) return false;
  walls = &chunk->parts[chunk->part_count - 2]; /* the roof's addition may have moved the array */
  float h = building->height;
  for (size_t i = 0; i < count; i++) {
    local_point_t a = building->points[i], b = building->points[(i + 1) % count];
    vec2_t along = v2_sub(b, a);
    double length = v2_length(along);
    vec2_t direction = v2_normalize(along);
    /* The ring is anticlockwise from above, so outside is to the right in
     * (x, north), which is (dn, dx) in (x, z) with dn = -dz. */
    vec3_t normal = vec3((float)-direction.z, 0, (float)direction.x);
    float u = (float)(length / o->window_width), v = h / o->storey_height;
    uint32_t base = (uint32_t)walls->mesh.vertex_count;
    if (!add_vertex(&walls->mesh, at(a, 0), normal, 0, 0) || !add_vertex(&walls->mesh, at(b, 0), normal, u, 0) ||
        !add_vertex(&walls->mesh, at(b, h), normal, u, v) || !add_vertex(&walls->mesh, at(a, h), normal, 0, v))
      return false;
    if (!add_face(&walls->mesh, base, base + 1, base + 2, normal) ||
        !add_face(&walls->mesh, base, base + 2, base + 3, normal))
      return false;
  }
  uint32_t* indices = malloc(3 * (count - 2) * sizeof(uint32_t));
  if (!indices) return false;
  bool clean;
  size_t triangles = triangulate_polygon(building->points, count, indices, &clean);
  if (!clean) {
    LOG_WARN("generate", "building %lld has an irregular footprint; roof approximated",
             (long long)building->source_id);
    stats->buildings_unclean++;
  }
  for (size_t i = 0; i < count; i++)
    if (!add_vertex(&roof->mesh, at(building->points[i], h), vec3(0, 1, 0), (float)(building->points[i].x * 0.1),
                    (float)(building->points[i].z * 0.1)))
      return false;
  for (size_t t = 0; t < triangles; t++)
    if (!add_face(&roof->mesh, indices[t * 3], indices[t * 3 + 1], indices[t * 3 + 2], vec3(0, 1, 0))) return false;
  free(indices);
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
      city_part_t* part = city_chunk_add_part(chunk, "grass", PART_COLLIDER_MESH, COLLISION_LAYER_GROUND,
                                              o->ground_friction, 0);
      if (!part) return false;
      float x0 = (float)(layout->min_x + column * o->chunk_size), x1 = x0 + o->chunk_size;
      float z0 = (float)(layout->min_z + row * o->chunk_size), z1 = z0 + o->chunk_size;
      float s = o->ground_uv_scale;
      uint32_t base = (uint32_t)part->mesh.vertex_count;
      if (!add_vertex(&part->mesh, vec3(x0, 0, z0), vec3(0, 1, 0), x0 * s, z0 * s) ||
          !add_vertex(&part->mesh, vec3(x1, 0, z0), vec3(0, 1, 0), x1 * s, z0 * s) ||
          !add_vertex(&part->mesh, vec3(x1, 0, z1), vec3(0, 1, 0), x1 * s, z1 * s) ||
          !add_vertex(&part->mesh, vec3(x0, 0, z1), vec3(0, 1, 0), x0 * s, z1 * s))
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

static void choose_spawn(city_package_t* package, const city_model_t* model, const city_generator_options_t* o) {
  const city_road_t* best = NULL;
  double best_length = 0;
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    if (road->class == ROAD_CLASS_SERVICE || road->class == ROAD_CLASS_MOTORWAY) continue;
    double length = 0;
    for (size_t i = 1; i < road->point_count; i++) length += v2_length(v2_sub(road->points[i], road->points[i - 1]));
    if (length > best_length) {
      best_length = length;
      best = road;
    }
  }
  if (!best) best = &model->roads[0];
  /* Midpoint of the longest segment of the chosen road, heading along it. */
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
  vec2_t direction = v2_normalize(v2_sub(b, a));
  package->spawn_position = vec3((float)((a.x + b.x) * 0.5), o->road_height + 0.9f, (float)((a.z + b.z) * 0.5));
  package->spawn_yaw = (float)atan2(-direction.x, -direction.z);
}

bool city_generate(city_package_t* package, const city_model_t* model, const city_generator_options_t* options,
                   const char* name, city_generator_stats_t* stats) {
  memset(stats, 0, sizeof(*stats));
  *package = city_package_create();
  if (model->road_count == 0) {
    LOG_ERROR("generate", "the model has no roads");
    return false;
  }
  snprintf(package->name, sizeof(package->name), "%s", name);
  snprintf(package->source, sizeof(package->source), "%s", model->source);
  snprintf(package->source_date, sizeof(package->source_date), "%s", model->import_date);
  time_t now = time(NULL);
  strftime(package->generated, sizeof(package->generated), "%Y-%m-%d", gmtime(&now));
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
      min_x = fmin(min_x, p.x), max_x = fmax(max_x, p.x), min_z = fmin(min_z, p.z), max_z = fmax(max_z, p.z);
    }
  for (size_t b = 0; b < model->building_count; b++)
    for (size_t i = 0; i < model->buildings[b].point_count; i++) {
      local_point_t p = model->buildings[b].points[i];
      min_x = fmin(min_x, p.x), max_x = fmax(max_x, p.x), min_z = fmin(min_z, p.z), max_z = fmax(max_z, p.z);
    }
  const double margin = 40.0;
  layout_t layout;
  layout.package = package;
  layout.options = options;
  layout.min_x = floor((min_x - margin) / options->chunk_size) * options->chunk_size;
  layout.min_z = floor((min_z - margin) / options->chunk_size) * options->chunk_size;
  layout.columns = (int)ceil((max_x + margin - layout.min_x) / options->chunk_size);
  layout.rows = (int)ceil((max_z + margin - layout.min_z) / options->chunk_size);
  if (layout.columns < 1) layout.columns = 1;
  if (layout.rows < 1) layout.rows = 1;

  size_t info_count;
  junction_info_t* infos = compute_junctions(model, options, &info_count);
  if (!infos) return false;
  bool ok = generate_ground(&layout, stats);
  for (size_t r = 0; ok && r < model->road_count; r++)
    ok = generate_road(&layout, &model->roads[r], infos, info_count, stats);
  for (size_t j = 0; ok && j < model->junction_count; j++)
    if (infos[j].patch) ok = generate_patch(&layout, &model->junctions[j], &infos[j], stats);
  for (size_t b = 0; ok && b < model->building_count; b++) ok = generate_building(&layout, &model->buildings[b], stats);
  free(infos);
  if (!ok) {
    LOG_ERROR("generate", "out of memory while generating");
    return false;
  }
  choose_spawn(package, model, options);
  LOG_INFO("generate", "%d x %d chunks, %zu roads, %zu junction patches, %zu pavements, %zu buildings "
           "(%zu approximated), %zu triangles",
           layout.columns, layout.rows, stats->roads, stats->junction_patches, stats->pavements, stats->buildings,
           stats->buildings_unclean, city_package_triangle_count(package));
  return true;
}
