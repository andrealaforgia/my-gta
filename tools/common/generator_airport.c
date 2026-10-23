/* An airport beside the city: a runway with its markings, a taxiway, an
 * apron with helicopter pads, a terminal, hangars and a tower, and the
 * aircraft that stand there. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "generator_internal.h"
#include "physics_world.h"

#define RUNWAY_HALF_LENGTH 450.0
#define RUNWAY_HALF_WIDTH 18.0
#define TAXIWAY_NEAR 52.0
#define TAXIWAY_FAR 64.0
#define APRON_FAR 124.0
#define AIRPORT_DEPTH 180.0 /* from the runway's middle, towards the city */
#define AIRPORT_BEHIND 40.0 /* and away from it */
#define AIRPORT_GAP 60.0    /* between the city's edge and the airport */
#define TILE 40.0
#define GATE_U 100.0 /* where the road from the city meets the apron */
#define GATE_W 120.0
#define GATE_NODE 9100000001ll
#define ACCESS_ROAD_SOURCE 9100

/* The airport's own frame: u along the runway, w towards the city. */
typedef struct {
  local_point_t origin;
  vec2_t axis;
  vec2_t toward;
} frame_t;

static local_point_t frame_point(const frame_t* f, double u, double w) {
  local_point_t p = {f->origin.x + f->axis.x * u + f->toward.x * w,
                     f->origin.z + f->axis.z * u + f->toward.z * w};
  return p;
}

static bool add_quad(city_part_t* part, local_point_t corners[4], float y,
                     float uv_scale) {
  uint32_t base = (uint32_t)part->mesh.vertex_count;
  vec3_t up = vec3(0, 1, 0);
  for (int i = 0; i < 4; i++)
    if (!gen_add_vertex(&part->mesh, gen_at(corners[i], y), up,
                        (float)(corners[i].x * uv_scale),
                        (float)(corners[i].z * uv_scale)))
      return false;
  return gen_add_face(&part->mesh, base, base + 1, base + 2, up) &&
         gen_add_face(&part->mesh, base, base + 2, base + 3, up);
}

/* A flat rectangle in the airport's frame, cut into tiles so each lands in
 * the chunk it lies over. */
static bool paint(layout_t* layout, const frame_t* f, const char* material,
                  part_collider_t collider, float friction, double u0,
                  double u1, double w0, double w1, float y, double max_tile) {
  int nu = (int)ceil((u1 - u0) / max_tile),
      nw = (int)ceil((w1 - w0) / max_tile);
  if (nu < 1) nu = 1;
  if (nw < 1) nw = 1;
  for (int i = 0; i < nu; i++)
    for (int j = 0; j < nw; j++) {
      double a0 = u0 + (u1 - u0) * i / nu, a1 = u0 + (u1 - u0) * (i + 1) / nu;
      double b0 = w0 + (w1 - w0) * j / nw, b1 = w0 + (w1 - w0) * (j + 1) / nw;
      local_point_t corners[4] = {
          frame_point(f, a0, b0), frame_point(f, a1, b0),
          frame_point(f, a1, b1), frame_point(f, a0, b1)};
      local_point_t middle = frame_point(f, (a0 + a1) / 2, (b0 + b1) / 2);
      city_chunk_t* chunk = gen_chunk_at(layout, middle);
      city_part_t* part = city_chunk_add_part(
          chunk, material, collider, COLLISION_LAYER_GROUND, friction, 0);
      if (!part || !add_quad(part, corners, y, 0.25f)) return false;
      gen_finish_part(chunk, part);
    }
  return true;
}

static bool paint_surface(layout_t* layout, const frame_t* f,
                          const char* material, double u0, double u1, double w0,
                          double w1, float y) {
  const city_generator_options_t* o = layout->options;
  float friction =
      strcmp(material, "paving") == 0 ? o->pavement_friction : o->road_friction;
  return paint(layout, f, material, PART_COLLIDER_MESH, friction, u0, u1, w0,
               w1, y, TILE);
}

static bool paint_line(layout_t* layout, const frame_t* f, const char* material,
                       double u0, double u1, double w0, double w1, float y) {
  return paint(layout, f, material, PART_COLLIDER_NONE, 0, u0, u1, w0, w1, y,
               TILE);
}

/* A building: four walls and a flat roof over a rectangle in the frame. */
static bool add_block(layout_t* layout, const frame_t* f, const char* facade,
                      double u0, double u1, double w0, double w1, float height,
                      uint64_t source) {
  const city_generator_options_t* o = layout->options;
  local_point_t corner[4] = {frame_point(f, u0, w0), frame_point(f, u1, w0),
                             frame_point(f, u1, w1), frame_point(f, u0, w1)};
  local_point_t middle = frame_point(f, (u0 + u1) / 2, (w0 + w1) / 2);
  city_chunk_t* chunk = gen_chunk_at(layout, middle);
  float ground = (float)gen_terrain_at(layout, middle.x, middle.z);
  city_part_t* walls = city_chunk_add_part(chunk, facade, PART_COLLIDER_MESH,
                                           COLLISION_LAYER_BUILDING,
                                           o->building_friction, source);
  city_part_t* roof =
      walls ? city_chunk_add_part(chunk, "roof_flat", PART_COLLIDER_MESH,
                                  COLLISION_LAYER_BUILDING,
                                  o->building_friction, source)
            : NULL;
  if (!walls || !roof) return false;
  walls = &chunk->parts[chunk->part_count - 2];
  roof = &chunk->parts[chunk->part_count - 1];
  for (int i = 0; i < 4; i++) {
    local_point_t p = corner[i], q = corner[(i + 1) % 4];
    double length = hypot(q.x - p.x, q.z - p.z);
    vec2_t d = gen_v2_normalize(gen_v2_sub(q, p));
    /* Outward is away from the middle. */
    vec2_t out = gen_v2_right(d);
    if (out.x * (p.x - middle.x) + out.z * (p.z - middle.z) < 0)
      out.x = -out.x, out.z = -out.z;
    vec3_t normal = vec3((float)out.x, 0, (float)out.z);
    if (!gen_add_wall_quad(walls, p, q, ground, ground + height, normal, 0,
                           (float)(length / o->window_width), 0,
                           height / o->storey_height))
      return false;
  }
  /* Roof as two triangles over the corners. */
  uint32_t base = (uint32_t)roof->mesh.vertex_count;
  for (int i = 0; i < 4; i++)
    if (!gen_add_vertex(&roof->mesh, gen_at(corner[i], ground + height),
                        vec3(0, 1, 0), (float)(corner[i].x * 0.2),
                        (float)(corner[i].z * 0.2)))
      return false;
  if (!gen_add_face(&roof->mesh, base, base + 1, base + 2, vec3(0, 1, 0)) ||
      !gen_add_face(&roof->mesh, base, base + 2, base + 3, vec3(0, 1, 0)))
    return false;
  gen_finish_part(chunk, walls);
  gen_finish_part(chunk, roof);
  return true;
}

static bool runway_markings(layout_t* layout, const frame_t* f) {
  float y = layout->options->road_height + 0.012f;
  /* Edge lines. */
  for (int side = -1; side <= 1; side += 2) {
    double w = side * (RUNWAY_HALF_WIDTH - 1.2);
    if (!paint_line(layout, f, "marking", -RUNWAY_HALF_LENGTH + 4,
                    RUNWAY_HALF_LENGTH - 4, w - 0.2, w + 0.2, y))
      return false;
  }
  /* The centre line, in dashes. */
  for (double u = -RUNWAY_HALF_LENGTH + 60; u < RUNWAY_HALF_LENGTH - 60;
       u += 30.0)
    if (!paint_line(layout, f, "marking", u, u + 15.0, -0.3, 0.3, y))
      return false;
  /* Thresholds: a row of bars across each end. */
  for (int end = -1; end <= 1; end += 2)
    for (int bar = -5; bar <= 5; bar++) {
      if (bar == 0) continue;
      double w = bar * 2.8;
      double u0 = end * (RUNWAY_HALF_LENGTH - 8.0);
      double u1 = end * (RUNWAY_HALF_LENGTH - 32.0);
      if (!paint_line(layout, f, "marking", fmin(u0, u1), fmax(u0, u1), w - 0.6,
                      w + 0.6, y))
        return false;
    }
  /* Touchdown bars either side of the centre line, 300 m in. */
  for (int end = -1; end <= 1; end += 2)
    for (int side = -1; side <= 1; side += 2) {
      double u0 = end * (RUNWAY_HALF_LENGTH - 300.0);
      double u1 = end * (RUNWAY_HALF_LENGTH - 322.0);
      double w = side * 7.0;
      if (!paint_line(layout, f, "marking", fmin(u0, u1), fmax(u0, u1), w - 1.0,
                      w + 1.0, y))
        return false;
    }
  return true;
}

/* An H in a square on a helicopter pad. */
static bool helipad(layout_t* layout, const frame_t* f, double u, double w) {
  float y = layout->options->road_height + 0.03f;
  const double h = 11.0;
  if (!paint_surface(layout, f, "parking", u - h, u + h, w - h, w + h,
                     layout->options->road_height + 0.02f))
    return false;
  y += 0.01f;
  return paint_line(layout, f, "marking", u - 4.0, u + 4.0, w - 4.2, w - 3.2,
                    y) &&
         paint_line(layout, f, "marking", u - 4.0, u + 4.0, w + 3.2, w + 4.2,
                    y) &&
         paint_line(layout, f, "marking", u - 0.5, u + 0.5, w - 4.2, w + 4.2,
                    y) &&
         paint_line(layout, f, "marking", u - h + 1.0, u + h - 1.0, w - h + 1.0,
                    w - h + 1.4, y) &&
         paint_line(layout, f, "marking", u - h + 1.0, u + h - 1.0, w + h - 1.4,
                    w + h - 1.0, y);
}

static bool stand_aircraft(layout_t* layout, const frame_t* f, int kind,
                           double u, double w) {
  city_package_t* package = layout->package;
  if (package->aircraft_count >= CITY_PACKAGE_MAX_AIRCRAFT) return true;
  local_point_t at = frame_point(f, u, w);
  /* Noses along the runway, so the first turn is onto a link. */
  city_aircraft_t* craft = &package->aircraft[package->aircraft_count++];
  craft->kind = kind;
  craft->x = (float)at.x;
  craft->y = (float)gen_terrain_at(layout, at.x, at.z) +
             layout->options->road_height +
             (kind == AIRCRAFT_PLANE ? 0.02f : 0.1f);
  craft->z = (float)at.z;
  craft->yaw = (float)atan2(-f->axis.x, -f->axis.z);
  return true;
}

static bool generate_at(layout_t* layout, const frame_t* f) {
  float road = layout->options->road_height;
  /* Grass all round, as an airfield has. */
  if (!paint(layout, f, "park", PART_COLLIDER_NONE, 0, -RUNWAY_HALF_LENGTH - 30,
             RUNWAY_HALF_LENGTH + 30, -AIRPORT_BEHIND, AIRPORT_DEPTH, 0.025f,
             TILE))
    return false;
  /* Runway, taxiway and its links. */
  if (!paint_surface(layout, f, "parking", -RUNWAY_HALF_LENGTH,
                     RUNWAY_HALF_LENGTH, -RUNWAY_HALF_WIDTH, RUNWAY_HALF_WIDTH,
                     road) ||
      !paint_surface(layout, f, "parking", -400, 400, TAXIWAY_NEAR, TAXIWAY_FAR,
                     road))
    return false;
  static const double links[3] = {-390, -6, 378};
  for (int i = 0; i < 3; i++)
    if (!paint_surface(layout, f, "parking", links[i], links[i] + 12,
                       RUNWAY_HALF_WIDTH, TAXIWAY_NEAR, road))
      return false;
  if (!runway_markings(layout, f)) return false;
  /* Yellow along the taxiway and down each link. */
  float y = road + 0.012f;
  if (!paint_line(layout, f, "marking_yellow", -400, 400, 57.85, 58.15, y))
    return false;
  for (int i = 0; i < 3; i++)
    if (!paint_line(layout, f, "marking_yellow", links[i] + 5.85,
                    links[i] + 6.15, RUNWAY_HALF_WIDTH, TAXIWAY_NEAR, y))
      return false;
  /* The apron, helicopter pads, then the buildings. */
  if (!paint_surface(layout, f, "paving", -140, 140, TAXIWAY_FAR, APRON_FAR,
                     road + 0.012f) ||
      !helipad(layout, f, 75, 100) || !helipad(layout, f, 115, 100))
    return false;
  if (!add_block(layout, f, "facade_fresco", -60, 60, 135, 153, 9.0f, 9001) ||
      !add_block(layout, f, "hangar", -180, -140, 130, 170, 12.0f, 9002) ||
      !add_block(layout, f, "hangar", -135, -95, 130, 170, 12.0f, 9003) ||
      !add_block(layout, f, "facade_brick", 75, 83, 135, 143, 24.0f, 9004))
    return false;
  for (int i = 0; i < 3; i++)
    if (!stand_aircraft(layout, f, AIRCRAFT_PLANE, -100.0 + 50.0 * i, 92))
      return false;
  return stand_aircraft(layout, f, AIRCRAFT_HELICOPTER, 75, 100) &&
         stand_aircraft(layout, f, AIRCRAFT_HELICOPTER, 115, 100);
}

static bool in_water(const city_model_t* model, local_point_t p) {
  for (size_t a = 0; a < model->area_count; a++)
    if (model->areas[a].kind == AREA_WATER &&
        gen_point_inside(p, model->areas[a].points,
                         model->areas[a].point_count))
      return true;
  return false;
}

/* Whether any part of the airport's footprint is sea, river or lake. */
static bool footprint_is_wet(const city_model_t* model, const frame_t* f) {
  for (double u = -RUNWAY_HALF_LENGTH - 30; u <= RUNWAY_HALF_LENGTH + 30;
       u += 30)
    for (double w = -AIRPORT_BEHIND; w <= AIRPORT_DEPTH; w += 30)
      if (in_water(model, frame_point(f, u, w))) return true;
  return false;
}

/* Beside which edge of the city the airport goes: the side asked for, else
 * the first dry one of south, west, east and north. */
static bool choose_frame(const city_model_t* model, const char* side_option,
                         double min_x, double min_z, double max_x, double max_z,
                         frame_t* out) {
  static const char sides[4] = {'S', 'W', 'E', 'N'};
  double mid_x = (min_x + max_x) / 2, mid_z = (min_z + max_z) / 2;
  for (int attempt = 0; attempt < 4; attempt++) {
    char side = sides[attempt];
    if (side_option && side_option[0] && attempt == 0)
      side = side_option[0];
    else if (side_option && side_option[0] && sides[attempt] == side_option[0])
      continue;
    frame_t f;
    f.origin.x = mid_x, f.origin.z = mid_z;
    double reach = AIRPORT_DEPTH + AIRPORT_GAP;
    switch (side) {
      case 'N':
        f.toward = gen_v2(0, 1), f.axis = gen_v2(1, 0);
        f.origin = (local_point_t){mid_x, min_z - reach};
        break;
      case 'E':
        f.toward = gen_v2(-1, 0), f.axis = gen_v2(0, 1);
        f.origin = (local_point_t){max_x + reach, mid_z};
        break;
      case 'W':
        f.toward = gen_v2(1, 0), f.axis = gen_v2(0, 1);
        f.origin = (local_point_t){min_x - reach, mid_z};
        break;
      default:
        f.toward = gen_v2(0, -1), f.axis = gen_v2(1, 0);
        f.origin = (local_point_t){mid_x, max_z + reach};
        break;
    }
    if (!footprint_is_wet(model, &f)) {
      *out = f;
      return true;
    }
  }
  return false;
}

bool gen_airport_site(const city_model_t* model, const char* side, double min_x,
                      double min_z, double max_x, double max_z,
                      airport_site_t* site) {
  frame_t f;
  memset(site, 0, sizeof(*site));
  if (side && side[0] == '-') return false; /* none wanted */
  if (!choose_frame(model, side, min_x, min_z, max_x, max_z, &f)) return false;
  site->present = true;
  site->gate = frame_point(&f, GATE_U, GATE_W);
  site->origin = f.origin;
  site->axis = f.axis;
  site->toward = f.toward;
  site->lo_x = site->lo_z = 1e300;
  site->hi_x = site->hi_z = -1e300;
  for (int i = 0; i < 4; i++) {
    double u = (i & 1 ? 1 : -1) * (RUNWAY_HALF_LENGTH + 30);
    double w = i & 2 ? AIRPORT_DEPTH : -AIRPORT_BEHIND;
    local_point_t p = frame_point(&f, u, w);
    site->lo_x = fmin(site->lo_x, p.x), site->hi_x = fmax(site->hi_x, p.x);
    site->lo_z = fmin(site->lo_z, p.z), site->hi_z = fmax(site->hi_z, p.z);
  }
  return true;
}

bool generate_airport(layout_t* layout, const airport_site_t* site) {
  if (!site->present) return true;
  frame_t f = {site->origin, site->axis, site->toward};
  if (!generate_at(layout, &f)) return false;
  city_package_t* package = layout->package;
  package->airport.present = true;
  package->airport.x = (float)site->origin.x;
  package->airport.z = (float)site->origin.z;
  package->airport.yaw = (float)atan2(-site->axis.x, -site->axis.z);
  package->airport.length = (float)(2 * RUNWAY_HALF_LENGTH);
  package->airport.width = (float)(2 * RUNWAY_HALF_WIDTH);
  package->airport.gate_x = (float)site->gate.x;
  package->airport.gate_y =
      (float)gen_terrain_at(layout, site->gate.x, site->gate.z) +
      layout->options->road_height;
  package->airport.gate_z = (float)site->gate.z;
  return true;
}

static bool drivable_for_access(const city_road_t* road) {
  return road->class != ROAD_CLASS_PEDESTRIAN &&
         road->class != ROAD_CLASS_PATH && road->class != ROAD_CLASS_WATERWAY &&
         road->class != ROAD_CLASS_MOTORWAY && !road->tunnel && !road->bridge;
}

bool gen_airport_road(const city_model_t* model, const airport_site_t* site,
                      city_model_t* out) {
  *out = *model;
  if (!site->present) return false;
  /* The nearest end of a street a car can use to the airport's front. */
  frame_t f = {site->origin, site->axis, site->toward};
  local_point_t front = frame_point(&f, GATE_U, AIRPORT_DEPTH + 10.0);
  const city_road_t* best = NULL;
  size_t best_end = 0;
  double best_distance = 1e300;
  for (size_t r = 0; r < model->road_count; r++) {
    const city_road_t* road = &model->roads[r];
    if (!drivable_for_access(road) || road->point_count < 2) continue;
    for (int end = 0; end < 2; end++) {
      size_t i = end ? road->point_count - 1 : 0;
      double d =
          hypot(road->points[i].x - front.x, road->points[i].z - front.z);
      if (d < best_distance && road->node_ids[i] != 0)
        best = road, best_end = i, best_distance = d;
    }
  }
  if (!best) return false;
  local_point_t start = best->points[best_end];
  size_t junction = 0;
  while (junction < model->junction_count &&
         model->junctions[junction].node_id != best->node_ids[best_end])
    junction++;
  if (junction == model->junction_count) return false;

  out->roads = malloc((model->road_count + 1) * sizeof(city_road_t));
  out->junctions =
      malloc((model->junction_count + 1) * sizeof(city_junction_t));
  if (!out->roads || !out->junctions) {
    free(out->roads);
    free(out->junctions);
    *out = *model;
    return false;
  }
  memcpy(out->roads, model->roads, model->road_count * sizeof(city_road_t));
  memcpy(out->junctions, model->junctions,
         model->junction_count * sizeof(city_junction_t));
  out->junctions[junction].degree++;
  city_junction_t* gate = &out->junctions[out->junction_count++];
  gate->node_id = GATE_NODE;
  gate->position = site->gate;
  gate->degree = 1;

  city_road_t* road = &out->roads[out->road_count++];
  memset(road, 0, sizeof(*road));
  road->source_id = ACCESS_ROAD_SOURCE;
  road->class = ROAD_CLASS_SECONDARY;
  road->width = 8.0f;
  road->width_inferred = false;
  snprintf(road->name, sizeof(road->name), "Airport_Road");
  road->point_count = 3;
  road->points = calloc(3, sizeof(local_point_t));
  road->node_ids = calloc(3, sizeof(int64_t));
  if (!road->points || !road->node_ids) {
    free(road->points);
    free(road->node_ids);
    free(out->roads);
    free(out->junctions);
    *out = *model;
    return false;
  }
  road->points[0] = start;
  road->points[1] = frame_point(&f, GATE_U, AIRPORT_DEPTH + 10.0);
  road->points[2] = site->gate;
  road->node_ids[0] = best->node_ids[best_end];
  road->node_ids[2] = GATE_NODE;
  return true;
}

void gen_airport_road_free(city_model_t* out) {
  if (out->road_count == 0) return;
  city_road_t* added = &out->roads[out->road_count - 1];
  free(added->points);
  free(added->node_ids);
  free(out->roads);
  free(out->junctions);
  memset(out, 0, sizeof(*out));
}
