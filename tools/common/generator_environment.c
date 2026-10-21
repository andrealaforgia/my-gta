/* Ground, parks, trees, street furniture and the terrain. */

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "config.h"
#include "generator_internal.h"
#include "junction_geometry.h"
#include "log.h"
#include "physics_world.h"
#include "triangulate.h"

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
    if (!gen_add_vertex(&part->mesh, gen_at(points[i], y), vec3(0, 1, 0),
                        (float)(points[i].x * uv_scale),
                        (float)(points[i].z * uv_scale))) {
      free(indices);
      return false;
    }
  for (size_t t = 0; t < triangles; t++) {
    local_point_t tri[3] = {points[indices[t * 3]], points[indices[t * 3 + 1]],
                            points[indices[t * 3 + 2]]};
    if (fabs(polygon_signed_area(tri, 3)) < 1e-4) continue;
    if (!gen_add_face(&part->mesh, base + indices[t * 3],
                      base + indices[t * 3 + 1], base + indices[t * 3 + 2],
                      vec3(0, 1, 0))) {
      free(indices);
      return false;
    }
  }
  free(indices);
  gen_finish_part(chunk, part);
  return true;
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
      gen_chunk_at(layout, gen_centroid(area->points, area->point_count));
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
      bool fits = gen_point_inside(p, area->points, area->point_count);
      for (int k = 0; k < 4 && fits; k++)
        fits = gen_point_inside(margin[k], area->points, area->point_count);
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
  gen_finish_part(chunk, trunks);
  gen_finish_part(chunk, leaves);
  stats->trees += (size_t)planted;
  return true;
}

/* ---- street furniture ---- */

/* The direction of the nearest road to a point, a unit vector, and how far
 * it is. */
static vec2_t nearest_road_direction(const city_model_t* model, local_point_t p,
                                     double* distance) {
  double best = 1e30;
  vec2_t direction = gen_v2(1, 0);
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    if (road->class == ROAD_CLASS_WATERWAY) continue;
    for (size_t k = 1; k < road->point_count; k++) {
      double d =
          gen_point_segment_distance(p, road->points[k - 1], road->points[k]);
      if (d < best) {
        best = d;
        direction =
            gen_v2_normalize(gen_v2_sub(road->points[k], road->points[k - 1]));
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

/* Zebra stripes at a mapped crossing node on a street cars use, unless a
 * junction is close enough to have its own. */
bool generate_crossings(layout_t* layout, const city_model_t* model,
                        const road_fit_t* fits) {
  const city_generator_options_t* o = layout->options;
  for (size_t f = 0; f < model->furniture_count; f++) {
    if (model->furniture[f].kind != OSM_FURNITURE_CROSSING) continue;
    local_point_t p = {model->furniture[f].x, model->furniture[f].z};
    size_t best_road = model->road_count;
    double best = 2.0;
    vec2_t direction = gen_v2(1, 0);
    for (size_t r = 0; r < model->road_count; r++) {
      const city_road_t* road = &model->roads[r];
      if (road->class == ROAD_CLASS_PATH ||
          road->class == ROAD_CLASS_WATERWAY ||
          road->class == ROAD_CLASS_PEDESTRIAN ||
          road->class == ROAD_CLASS_MOTORWAY)
        continue;
      for (size_t k = 1; k < road->point_count; k++) {
        double d =
            gen_point_segment_distance(p, road->points[k - 1], road->points[k]);
        if (d < best) {
          best = d, best_road = r;
          direction = gen_v2_normalize(
              gen_v2_sub(road->points[k], road->points[k - 1]));
        }
      }
    }
    if (best_road == model->road_count) continue;
    bool near_junction = false;
    for (size_t j = 0; j < model->junction_count && !near_junction; j++)
      near_junction = hypot(model->junctions[j].position.x - p.x,
                            model->junctions[j].position.z - p.z) < 14.0 &&
                      model->junctions[j].degree >= 3;
    if (near_junction) continue;
    double half = fits[best_road].half_width;
    city_chunk_t* chunk = gen_chunk_at(layout, p);
    city_part_t* part =
        city_chunk_add_part(chunk, "marking", PART_COLLIDER_NONE, 0, 0, 0);
    if (!part) return false;
    float y = o->road_height + 0.004f;
    for (double across = -half + 0.4; across + 0.5 <= half - 0.4; across += 1.0)
      if (!gen_add_marking_quad(&part->mesh, p, direction, -1.2, 1.2, across,
                                across + 0.5, y))
        return false;
    if (part->mesh.vertex_count == 0) {
      mesh_data_free(&part->mesh);
      chunk->part_count--;
    } else {
      gen_finish_part(chunk, part);
      /* People cross here; cars give way to them. */
      if (!city_package_add_crossing(layout->package, (float)p.x, (float)p.z))
        return false;
    }
  }
  return true;
}

/* A traffic signal on a pole: the pole and head are solid, the three lenses
 * are parts of their own that the game lights in turn. `along` is the way
 * the head faces (both ways when `both_faces`); group 1 uses the second set
 * of lens materials so the two phases light independently. */
bool gen_add_signal_head(layout_t* layout, vec3_t c, vec2_t along,
                         bool both_faces, int group, uint64_t source_id) {
  local_point_t at = {c.x, c.z};
  city_chunk_t* chunk = gen_chunk_at(layout, at);
  source_id = gen_pole_source(layout, source_id);
  static const char* names[2][3] = {
      {"signal_red", "signal_amber", "signal_green"},
      {"signal_r2", "signal_a2", "signal_g2"}};
  const char* const* lens_names = names[group ? 1 : 0];
  city_part_t* head =
      city_chunk_add_part(chunk, "furniture", PART_COLLIDER_MESH,
                          COLLISION_LAYER_PROP, 0.5f, source_id);
  if (!head) return false;
  for (int i = 0; i < 3; i++)
    if (!city_chunk_add_part(chunk, lens_names[i], PART_COLLIDER_NONE, 0, 0,
                             source_id))
      return false;
  size_t first = chunk->part_count - 4;
  head = &chunk->parts[first];
  city_part_t* lens[3] = {&chunk->parts[first + 1], &chunk->parts[first + 2],
                          &chunk->parts[first + 3]};
  if (!mesh_data_add_cylinder_y(&head->mesh, c, 0.06f, 3.4f, 6) ||
      !add_oriented_box(&head->mesh, vec3_add(c, vec3(0, 3.1f, 0)),
                        vec3(0.17f, 0.45f, 0.17f), along))
    return false;
  /* Three lenses on each face looking along the road. */
  for (int face = both_faces ? -1 : 1; face <= 1; face += 2) {
    vec3_t out =
        vec3(face * (float)along.x * 0.19f, 0, face * (float)along.z * 0.19f);
    vec3_t size = vec3(0.11f * (float)(fabs(along.z) + 0.2), 0.11f,
                       0.11f * (float)(fabs(along.x) + 0.2));
    const float heights[3] = {3.35f, 3.1f, 2.85f};
    for (int i = 0; i < 3; i++)
      if (!mesh_data_add_box(&lens[i]->mesh,
                             vec3_add(vec3_add(c, vec3(0, heights[i], 0)), out),
                             size))
        return false;
  }
  gen_finish_part(chunk, lens[0]);
  gen_finish_part(chunk, lens[1]);
  gen_finish_part(chunk, lens[2]);
  gen_finish_part(chunk, head);
  return true;
}

bool gen_register_signal(layout_t* layout, local_point_t at, vec2_t heading,
                         int group) {
  city_signal_t signal = {(float)at.x, (float)at.z, (float)heading.x,
                          (float)heading.z, group};
  return city_package_add_signal(layout->package, signal);
}

bool generate_furniture(layout_t* layout, const city_model_t* model,
                        city_generator_stats_t* stats) {
  (void)stats;
  for (size_t f = 0; f < model->furniture_count; f++) {
    local_point_t p = {model->furniture[f].x, model->furniture[f].z};
    int kind = model->furniture[f].kind;
    if (kind == OSM_FURNITURE_CROSSING) continue; /* painted, not built */
    double road_distance;
    vec2_t along = nearest_road_direction(model, p, &road_distance);
    if (kind == OSM_FURNITURE_SIGNALS) {
      /* A mapped head on a road faces both ways along it. */
      int group = fabs(along.x) >= fabs(along.z) ? 0 : 1;
      if (!gen_add_signal_head(
              layout,
              vec3((float)p.x, layout->options->kerb_height, (float)p.z), along,
              true, group, 0) ||
          !gen_register_signal(layout, p, along, group) ||
          !gen_register_signal(layout, p, gen_v2(-along.x, -along.z), group))
        return false;
      continue;
    }
    city_chunk_t* chunk = gen_chunk_at(layout, p);
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
      if (!city_package_add_bus_stop(layout->package, (float)p.x, (float)p.z))
        return false;
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
    }
    if (!ok) return false;
    gen_finish_part(chunk, part);
  }
  return true;
}

/* ---- walls, fences and hedges ---- */

/* Whether a point lies on the carriageway of a street, where a barrier
 * would block it; mapped walls end at gates, which are not built. */
static bool on_carriageway(const city_model_t* model, local_point_t p) {
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    if (road->class == ROAD_CLASS_WATERWAY) continue;
    double reach = road->width * 0.5 + 0.3;
    for (size_t k = 1; k < road->point_count; k++)
      if (gen_point_segment_distance(p, road->points[k - 1], road->points[k]) <
          reach)
        return true;
  }
  return false;
}

/* One slab from p to q: both faces and the top. */
static bool add_barrier_piece(city_part_t* part, local_point_t p,
                              local_point_t q, float y0, float height,
                              double thickness) {
  vec2_t side = gen_v2_right(gen_v2_normalize(gen_v2_sub(q, p)));
  vec3_t normal = vec3((float)side.x, 0, (float)side.z);
  local_point_t pr = gen_offset(p, side, thickness * 0.5);
  local_point_t qr = gen_offset(q, side, thickness * 0.5);
  local_point_t pl = gen_offset(p, side, -thickness * 0.5);
  local_point_t ql = gen_offset(q, side, -thickness * 0.5);
  float y1 = y0 + height;
  float length = (float)gen_v2_length(gen_v2_sub(q, p));
  if (!gen_add_wall_quad(part, pr, qr, y0, y1, normal, 0, length, 0, height) ||
      !gen_add_wall_quad(part, ql, pl, y0, y1, vec3_scale(normal, -1), 0,
                         length, 0, height))
    return false;
  uint32_t base = (uint32_t)part->mesh.vertex_count;
  vec3_t up = vec3(0, 1, 0);
  return gen_add_vertex(&part->mesh, gen_at(pl, y1), up, 0, 0) &&
         gen_add_vertex(&part->mesh, gen_at(ql, y1), up, length, 0) &&
         gen_add_vertex(&part->mesh, gen_at(qr, y1), up, length, 1) &&
         gen_add_vertex(&part->mesh, gen_at(pr, y1), up, 0, 1) &&
         gen_add_face(&part->mesh, base, base + 1, base + 2, up) &&
         gen_add_face(&part->mesh, base, base + 2, base + 3, up);
}

bool generate_barriers(layout_t* layout, const city_model_t* model) {
  static const char* materials[BARRIER_KIND_COUNT] = {"wall", "fence", "hedge"};
  static const float heights[BARRIER_KIND_COUNT] = {1.9f, 1.25f, 1.3f};
  static const double thicknesses[BARRIER_KIND_COUNT] = {0.3, 0.07, 0.7};
  const double piece_length = 2.0;
  for (size_t b = 0; b < model->barrier_count; b++) {
    const city_barrier_t* barrier = &model->barriers[b];
    city_part_t* part = NULL;
    city_chunk_t* part_chunk = NULL;
    for (size_t k = 1; k < barrier->point_count; k++) {
      local_point_t a = barrier->points[k - 1], c = barrier->points[k];
      double length = gen_v2_length(gen_v2_sub(c, a));
      size_t pieces = (size_t)ceil(length / piece_length);
      if (pieces == 0) continue;
      for (size_t i = 0; i < pieces; i++) {
        double t0 = (double)i / (double)pieces;
        double t1 = (double)(i + 1) / (double)pieces;
        local_point_t p = {a.x + (c.x - a.x) * t0, a.z + (c.z - a.z) * t0};
        local_point_t q = {a.x + (c.x - a.x) * t1, a.z + (c.z - a.z) * t1};
        local_point_t middle = {(p.x + q.x) * 0.5, (p.z + q.z) * 0.5};
        if (on_carriageway(model, p) || on_carriageway(model, q) ||
            on_carriageway(model, middle))
          continue;
        city_chunk_t* chunk = gen_chunk_at(layout, middle);
        if (chunk != part_chunk) {
          part = city_chunk_add_part(chunk, materials[barrier->kind],
                                     PART_COLLIDER_MESH, COLLISION_LAYER_PROP,
                                     0.5f, (uint64_t)barrier->source_id);
          if (!part) return false;
          part_chunk = chunk;
        }
        if (!add_barrier_piece(part, p, q, layout->options->kerb_height - 0.05f,
                               heights[barrier->kind],
                               thicknesses[barrier->kind]))
          return false;
      }
    }
  }
  return true;
}

/* The mapped single trees, each a trunk and a crown like a park's, with a
 * collider on the trunk. They sit on the ground where they were mapped. */
bool generate_street_trees(layout_t* layout, const city_model_t* model,
                           city_generator_stats_t* stats) {
  if (layout->options->tree_spacing <= 0) return true;
  for (size_t t = 0; t < model->tree_count; t++) {
    local_point_t p = model->trees[t];
    city_chunk_t* chunk = gen_chunk_at(layout, p);
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
    gen_finish_part(chunk, trunks);
    gen_finish_part(chunk, leaves);
    stats->trees++;
  }
  return true;
}

bool generate_area(layout_t* layout, const city_area_t* area,
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
      gen_chunk_at(layout, gen_centroid(area->points, area->point_count));
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

bool generate_ground(layout_t* layout, city_generator_stats_t* stats) {
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
          float y = hilly ? (float)gen_terrain_at(layout, x, z) - 0.05f : 0.0f;
          if (!gen_add_vertex(&part->mesh, vec3(x, y, z), vec3(0, 1, 0), x * s,
                              z * s))
            return false;
        }
      for (int j = 0; j < cells; j++)
        for (int i = 0; i < cells; i++) {
          uint32_t a = base + (uint32_t)(j * (cells + 1) + i);
          uint32_t b = a + 1, d = a + (uint32_t)(cells + 1), c = d + 1;
          if (!gen_add_face(&part->mesh, a, b, c, vec3(0, 1, 0)) ||
              !gen_add_face(&part->mesh, a, c, d, vec3(0, 1, 0)))
            return false;
        }
      gen_finish_part(chunk, part);
      stats->ground_tiles++;
    }
  }
  return true;
}

/* Lays everything but the ground and the buildings (already placed) on
 * the terrain: large surfaces are cut into small triangles first so they
 * follow the hills, then every vertex rises by the ground height under
 * it. Chunk bounds are rebuilt. */
bool gen_apply_terrain(layout_t* layout) {
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
          position->y +=
              (float)gen_terrain_at(layout, position->x, position->z);
        }
      }
      chunk->bounds = aabb_merge(chunk->bounds, mesh_data_bounds(&part->mesh));
    }
  }
  return true;
}
