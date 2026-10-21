#include "vehicle_bodies.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char* style_names[BODY_STYLE_COUNT] = {
    "car",    "police",  "ambulance", "fire_engine", "bus",  "box_truck",
    "tanker", "flatbed", "refuse",    "pickup",      "mixer"};

body_style_t body_style_from_name(const char* name) {
  for (int i = 0; i < BODY_STYLE_COUNT; i++)
    if (strcmp(style_names[i], name) == 0) return (body_style_t)i;
  return BODY_CAR;
}

const char* body_style_name(body_style_t style) {
  return style < BODY_STYLE_COUNT ? style_names[style] : "car";
}

void body_build_init(body_build_t* out) {
  memset(out, 0, sizeof(*out));
  out->body = mesh_data_create();
  out->cabin = mesh_data_create();
  out->glass = mesh_data_create();
  out->lights = mesh_data_create();
  out->rear = mesh_data_create();
  out->trim = mesh_data_create();
  out->accent = mesh_data_create();
  out->beacon_a = mesh_data_create();
  out->beacon_b = mesh_data_create();
  out->beacon_a_colour = vec3(1.0f, 0.1f, 0.08f);
  out->beacon_b_colour = vec3(0.1f, 0.3f, 1.0f);
  out->accent_colour = vec3(0.9f, 0.9f, 0.88f);
}

void body_build_free(body_build_t* out) {
  mesh_data_free(&out->body);
  mesh_data_free(&out->cabin);
  mesh_data_free(&out->glass);
  mesh_data_free(&out->lights);
  mesh_data_free(&out->rear);
  mesh_data_free(&out->trim);
  mesh_data_free(&out->accent);
  mesh_data_free(&out->beacon_a);
  mesh_data_free(&out->beacon_b);
}

/* ---- shape helpers ---- */

static bool box(mesh_data_t* m, float x0, float y0, float z0, float x1,
                float y1, float z1) {
  return mesh_data_add_box(
      m, vec3((x0 + x1) * 0.5f, (y0 + y1) * 0.5f, (z0 + z1) * 0.5f),
      vec3((x1 - x0) * 0.5f, (y1 - y0) * 0.5f, (z1 - z0) * 0.5f));
}

/* A pane facing forward (-Z) at z. */
static bool front_pane(mesh_data_t* m, float x0, float x1, float y0, float y1,
                       float z) {
  return mesh_data_add_quad(m, vec3(x0, y0, z), vec3(x0, y1, z),
                            vec3(x1, y1, z), vec3(x1, y0, z), 1);
}

/* A pane facing back (+Z) at z. */
static bool rear_pane(mesh_data_t* m, float x0, float x1, float y0, float y1,
                      float z) {
  return mesh_data_add_quad(m, vec3(x1, y0, z), vec3(x1, y1, z),
                            vec3(x0, y1, z), vec3(x0, y0, z), 1);
}

/* A pane on a flank at x, between z_front (more negative) and z_back. */
static bool side_pane(mesh_data_t* m, int side, float x, float y0, float y1,
                      float z_front, float z_back) {
  if (side > 0)
    return mesh_data_add_quad(m, vec3(x, y0, z_back), vec3(x, y0, z_front),
                              vec3(x, y1, z_front), vec3(x, y1, z_back), 1);
  return mesh_data_add_quad(m, vec3(x, y0, z_front), vec3(x, y0, z_back),
                            vec3(x, y1, z_back), vec3(x, y1, z_front), 1);
}

/* A closed prism along Z about (cx, cy), `segments` sides. */
static bool prism_z(mesh_data_t* m, float cx, float cy, float r, float z0,
                    float z1, int segments) {
  vec3_t centre0 = vec3(cx, cy, z0), centre1 = vec3(cx, cy, z1);
  for (int k = 0; k < segments; k++) {
    float a0 = 2.0f * (float)M_PI * (float)k / (float)segments;
    float a1 = 2.0f * (float)M_PI * (float)(k + 1) / (float)segments;
    vec3_t p0 = vec3(cx + r * cosf(a0), cy + r * sinf(a0), 0);
    vec3_t p1 = vec3(cx + r * cosf(a1), cy + r * sinf(a1), 0);
    vec3_t a = vec3(p0.x, p0.y, z0), b = vec3(p1.x, p1.y, z0);
    vec3_t c = vec3(p1.x, p1.y, z1), d = vec3(p0.x, p0.y, z1);
    if (!mesh_data_add_quad(m, a, b, c, d, 1) ||
        !mesh_data_add_quad(m, centre0, a, b, b, 1) ||
        !mesh_data_add_quad(m, centre1, c, d, d, 1))
      return false;
  }
  return true;
}

/* ---- the parts lorries and vans share ---- */

typedef struct {
  float w, floor, top, nose, length; /* the cab: x +-w, y floor..top */
} cab_t;

/* A cab-over cab at the front: body, windscreen, side windows, lamps,
 * grille, bumper and mirrors. */
static bool add_cab(body_build_t* out, const cab_t* c) {
  float w = c->w, x0 = c->nose, x1 = c->nose + c->length;
  float glass_low = c->floor + (c->top - c->floor) * 0.45f;
  float glass_high = c->top - 0.12f;
  float gap = 0.012f;
  if (!box(&out->cabin, -w, c->floor, x0, w, c->top, x1)) return false;
  if (!front_pane(&out->glass, -w * 0.84f, w * 0.84f, glass_low, glass_high,
                  x0 - gap))
    return false;
  for (int side = -1; side <= 1; side += 2)
    if (!side_pane(&out->glass, side, side * (w + gap), glass_low, glass_high,
                   x0 + 0.15f, x0 + c->length * 0.72f))
      return false;
  for (int side = -1; side <= 1; side += 2) {
    if (!box(&out->lights, side * w * 0.72f - 0.14f, c->floor + 0.34f,
             x0 - 0.03f, side * w * 0.72f + 0.14f, c->floor + 0.5f, x0))
      return false;
    /* Mirrors on arms. */
    if (!box(&out->trim, side * (w + 0.02f) - 0.04f, glass_low + 0.1f,
             x0 + 0.25f, side * (w + 0.2f) + 0.04f, glass_low + 0.45f,
             x0 + 0.3f))
      return false;
  }
  return box(&out->trim, -w * 0.6f, c->floor + 0.2f, x0 - 0.04f, w * 0.6f,
             c->floor + 0.62f, x0) &&
         box(&out->trim, -w, c->floor - 0.04f, x0 - 0.14f, w, c->floor + 0.16f,
             x0);
}

/* The frame under the load. */
static bool add_frame(body_build_t* out, float w, float floor, float from,
                      float to) {
  return box(&out->trim, -w * 0.78f, floor - 0.3f, from, w * 0.78f, floor, to);
}

static bool add_tail_lights(body_build_t* out, float w, float y, float z) {
  for (int side = -1; side <= 1; side += 2)
    if (!box(&out->rear, side * w * 0.72f - 0.12f, y, z,
             side * w * 0.72f + 0.12f, y + 0.2f, z + 0.03f))
      return false;
  return true;
}

/* A pair of beacons across a roof at height y, z..z+0.3. */
static bool add_beacons(body_build_t* out, float w, float y, float z) {
  return box(&out->trim, -w * 0.5f, y, z, w * 0.5f, y + 0.05f, z + 0.3f) &&
         box(&out->beacon_a, -w * 0.5f, y + 0.05f, z, 0, y + 0.2f, z + 0.3f) &&
         box(&out->beacon_b, 0, y + 0.05f, z, w * 0.5f, y + 0.2f, z + 0.3f);
}

/* ---- styles ---- */

static bool build_bus(body_build_t* out, vec3_t h) {
  float w = h.x, L = h.z, floor = -h.y * 0.667f, belt = -0.05f;
  float roof = h.y - 0.45f, gap = 0.012f;
  out->own_paint = true;
  out->paint = vec3(0.88f, 0.88f, 0.84f);
  out->accent_colour = vec3(0.12f, 0.3f, 0.62f);
  /* Lower body, the roof and an air-conditioning pod. */
  if (!box(&out->body, -w, floor, -L, w, belt, L) ||
      !box(&out->body, -w, roof, -L, w, roof + 0.25f, L) ||
      !box(&out->body, -w * 0.7f, roof + 0.25f, -L * 0.3f, w * 0.7f, h.y,
           L * 0.45f))
    return false;
  /* Pillars between the windows. */
  for (float z = -L + 1.1f; z < L - 0.4f; z += 1.55f)
    if (!box(&out->body, -w - 0.01f, belt, z, w + 0.01f, roof, z + 0.1f))
      return false;
  /* Window band on both flanks, the windscreen and the rear window. */
  for (int side = -1; side <= 1; side += 2)
    if (!side_pane(&out->glass, side, side * (w + gap), belt + 0.05f,
                   roof - 0.05f, -L + 0.5f, L - 0.3f))
      return false;
  if (!front_pane(&out->glass, -w * 0.9f, w * 0.9f, belt + 0.05f, roof - 0.05f,
                  -L - gap) ||
      !rear_pane(&out->glass, -w * 0.7f, w * 0.7f, belt + 0.1f, roof - 0.15f,
                 L + gap))
    return false;
  /* The doors on the right flank, darker, and a stripe along both sides. */
  if (!box(&out->trim, w, floor + 0.1f, -L + 0.7f, w + 0.03f, roof - 0.05f,
           -L + 1.95f) ||
      !box(&out->trim, w, floor + 0.1f, 0.5f, w + 0.03f, roof - 0.05f, 1.75f))
    return false;
  for (int side = -1; side <= 1; side += 2)
    if (!box(&out->accent, side * w - 0.01f + (side > 0 ? 0.02f : -0.02f),
             floor + 0.45f, -L, side * w + (side > 0 ? 0.03f : -0.03f),
             floor + 0.72f, L))
      return false;
  /* Destination sign, lamps, bumpers, tail lights. */
  if (!box(&out->lights, -w * 0.5f, roof + 0.02f, -L - 0.02f, w * 0.5f,
           roof + 0.2f, -L) ||
      !box(&out->trim, -w, floor - 0.04f, -L - 0.12f, w, floor + 0.16f, -L) ||
      !box(&out->trim, -w, floor - 0.04f, L, w, floor + 0.16f, L + 0.12f))
    return false;
  for (int side = -1; side <= 1; side += 2)
    if (!box(&out->lights, side * w * 0.8f - 0.16f, floor + 0.3f, -L - 0.03f,
             side * w * 0.8f + 0.16f, floor + 0.48f, -L))
      return false;
  return add_tail_lights(out, w * 1.1f, floor + 0.35f, L);
}

static bool build_box_truck(body_build_t* out, vec3_t h) {
  float w = h.x, L = h.z, floor = -h.y * 0.667f;
  cab_t cab = {w, floor, h.y * 0.55f, -L, 2.0f};
  out->accent_colour = vec3(0.9f, 0.9f, 0.87f);
  return add_cab(out, &cab) && add_frame(out, w, floor, -L + 0.3f, L - 0.2f) &&
         box(&out->accent, -w, floor + 0.05f, -L + 2.15f, w, h.y, L) &&
         box(&out->trim, -w - 0.01f, floor + 0.05f, L - 0.05f, w + 0.01f,
             floor + 0.2f, L) &&
         add_tail_lights(out, w, floor + 0.3f, L);
}

static bool build_tanker(body_build_t* out, vec3_t h) {
  float w = h.x, L = h.z, floor = -h.y * 0.667f;
  cab_t cab = {w, floor, h.y * 0.55f, -L, 2.0f};
  float r = (h.y - floor - 0.25f) * 0.5f;
  out->accent_colour = vec3(0.78f, 0.8f, 0.82f);
  out->extra_axles = 1;
  out->extra_axle_z = 1.35f;
  return add_cab(out, &cab) && add_frame(out, w, floor, -L + 0.3f, L - 0.2f) &&
         prism_z(&out->accent, 0, floor + 0.25f + r, fminf(r, w), -L + 2.2f,
                 L - 0.1f, 16) &&
         box(&out->trim, -0.4f, h.y - 0.02f, -L * 0.2f, 0.4f, h.y + 0.12f,
             L * 0.2f) &&
         add_tail_lights(out, w, floor + 0.2f, L - 0.2f);
}

static bool build_flatbed(body_build_t* out, vec3_t h) {
  float w = h.x, L = h.z, floor = -h.y * 0.667f;
  cab_t cab = {w, floor, h.y * 0.55f, -L, 2.0f};
  out->accent_colour = vec3(0.5f, 0.38f, 0.22f);
  if (!add_cab(out, &cab) || !add_frame(out, w, floor, -L + 0.3f, L - 0.2f) ||
      !box(&out->accent, -w, floor, -L + 2.15f, w, floor + 0.18f, L))
    return false;
  /* A load of crates in the palette colour, and stakes round the deck. */
  float z = -L + 2.45f;
  for (int i = 0; z + 1.1f < L - 0.2f; i++, z += 1.25f)
    if (!box(&out->body, -w + 0.1f, floor + 0.18f, z, -0.05f,
             floor + 0.18f + 0.8f + 0.2f * (float)(i % 2), z + 1.1f) ||
        !box(&out->body, 0.05f, floor + 0.18f, z, w - 0.1f,
             floor + 0.18f + 0.9f - 0.2f * (float)(i % 2), z + 1.1f))
      return false;
  for (float zz = -L + 2.2f; zz < L; zz += 1.8f)
    for (int side = -1; side <= 1; side += 2)
      if (!box(&out->trim, side * w - 0.03f, floor + 0.18f, zz,
               side * w + 0.03f, floor + 0.8f, zz + 0.06f))
        return false;
  return add_tail_lights(out, w, floor + 0.0f, L);
}

static bool build_refuse(body_build_t* out, vec3_t h) {
  float w = h.x, L = h.z, floor = -h.y * 0.667f;
  cab_t cab = {w, floor, h.y * 0.55f, -L, 2.0f};
  out->own_paint = true;
  out->paint = vec3(0.1f, 0.42f, 0.2f);
  out->accent_colour = vec3(0.45f, 0.47f, 0.5f);
  out->extra_axles = 1;
  out->extra_axle_z = 1.3f;
  return add_cab(out, &cab) && add_frame(out, w, floor, -L + 0.3f, L - 0.2f) &&
         box(&out->body, -w, floor + 0.05f, -L + 2.15f, w, h.y - 0.25f,
             L - 0.55f) &&
         box(&out->body, -w * 0.96f, floor + 0.05f, L - 0.55f, w * 0.96f,
             h.y - 0.55f, L - 0.05f) &&
         box(&out->accent, -w * 0.8f, floor + 0.3f, L - 0.05f, w * 0.8f,
             floor + 1.1f, L + 0.2f) &&
         box(&out->accent, -w * 0.5f, h.y - 0.25f, -L + 2.4f, w * 0.5f,
             h.y - 0.1f, L - 0.8f) &&
         add_tail_lights(out, w, floor + 0.2f, L - 0.05f);
}

static bool build_pickup(body_build_t* out, vec3_t h) {
  float w = h.x, L = h.z, floor = -h.y * 0.667f;
  float top = h.y * 0.95f, deck = floor + 0.3f;
  float cab_len = 1.55f, bonnet = 0.95f;
  /* Bonnet, cab, then the open bed. */
  if (!box(&out->body, -w, floor, -L, w, floor + (top - floor) * 0.55f,
           -L + bonnet))
    return false;
  float x0 = -L + bonnet;
  float glass_low = floor + (top - floor) * 0.55f, glass_high = top - 0.1f;
  float gap = 0.012f;
  if (!box(&out->cabin, -w * 0.96f, glass_low - 0.05f, x0, w * 0.96f, top,
           x0 + cab_len) ||
      !box(&out->cabin, -w, floor, x0, w, glass_low, x0 + cab_len))
    return false;
  if (!mesh_data_add_quad(&out->glass, vec3(-w * 0.8f, glass_low, x0 - gap),
                          vec3(-w * 0.72f, glass_high, x0 + 0.3f - gap),
                          vec3(w * 0.72f, glass_high, x0 + 0.3f - gap),
                          vec3(w * 0.8f, glass_low, x0 - gap), 1))
    return false;
  for (int side = -1; side <= 1; side += 2)
    if (!side_pane(&out->glass, side, side * (w * 0.96f + gap), glass_low,
                   glass_high, x0 + 0.15f, x0 + cab_len - 0.2f))
      return false;
  if (!rear_pane(&out->glass, -w * 0.7f, w * 0.7f, glass_low + 0.05f,
                 glass_high, x0 + cab_len + gap))
    return false;
  /* Bed: floor and four walls. */
  float bed0 = x0 + cab_len + 0.05f;
  if (!box(&out->body, -w, deck - 0.1f, bed0, w, deck, L) ||
      !box(&out->body, -w, deck, bed0, -w + 0.06f, deck + 0.5f, L) ||
      !box(&out->body, w - 0.06f, deck, bed0, w, deck + 0.5f, L) ||
      !box(&out->body, -w, deck, L - 0.06f, w, deck + 0.5f, L) ||
      !box(&out->body, -w, deck, bed0, w, deck + 0.5f, bed0 + 0.06f))
    return false;
  for (int side = -1; side <= 1; side += 2)
    if (!box(&out->lights, side * w * 0.7f - 0.13f, floor + 0.3f, -L - 0.03f,
             side * w * 0.7f + 0.13f, floor + 0.46f, -L))
      return false;
  return box(&out->trim, -w * 0.6f, floor + 0.15f, -L - 0.04f, w * 0.6f,
             floor + 0.5f, -L) &&
         box(&out->trim, -w, floor - 0.05f, -L - 0.12f, w, floor + 0.12f, -L) &&
         box(&out->trim, -w, floor - 0.05f, L, w, floor + 0.12f, L + 0.1f) &&
         add_tail_lights(out, w, deck + 0.1f, L);
}

static bool build_mixer(body_build_t* out, vec3_t h) {
  float w = h.x, L = h.z, floor = -h.y * 0.667f;
  cab_t cab = {w, floor, h.y * 0.55f, -L, 2.0f};
  float r = (h.y - floor - 0.3f) * 0.5f;
  float cy = floor + 0.3f + r;
  out->accent_colour = vec3(0.92f, 0.55f, 0.06f);
  out->extra_axles = 1;
  out->extra_axle_z = 1.3f;
  /* The drum: a fat middle and a narrower mouth at the back. */
  return add_cab(out, &cab) && add_frame(out, w, floor, -L + 0.3f, L - 0.2f) &&
         prism_z(&out->accent, 0, cy, fminf(r, w), -L + 2.3f, L - 1.2f, 14) &&
         prism_z(&out->accent, 0, cy + 0.15f, fminf(r, w) * 0.55f, L - 1.2f, L,
                 12) &&
         add_tail_lights(out, w, floor + 0.2f, L - 0.2f);
}

static bool build_ambulance(body_build_t* out, vec3_t h) {
  float w = h.x, L = h.z, floor = -h.y * 0.667f;
  float top = h.y * 0.95f;
  cab_t cab = {w, floor, top * 0.8f, -L, 1.5f};
  out->own_paint = true;
  out->paint = vec3(0.93f, 0.93f, 0.92f);
  out->accent_colour = vec3(0.85f, 0.1f, 0.08f);
  out->beacon_a_colour = vec3(0.1f, 0.3f, 1.0f);
  if (!add_cab(out, &cab) || !add_frame(out, w, floor, -L + 0.3f, L - 0.2f) ||
      !box(&out->body, -w, floor, -L + 1.55f, w, top, L))
    return false;
  for (int side = -1; side <= 1; side += 2)
    if (!box(&out->accent, side * w - 0.02f + (side > 0 ? 0.03f : -0.03f),
             floor + 0.55f, -L + 1.55f, side * w + (side > 0 ? 0.04f : -0.04f),
             floor + 0.85f, L))
      return false;
  /* A cross on each flank. */
  for (int side = -1; side <= 1; side += 2) {
    float x = side * (w + 0.04f);
    float zc = 0.6f;
    if (!box(&out->accent, x - 0.01f, top - 0.9f, zc - 0.1f, x + 0.01f,
             top - 0.3f, zc + 0.1f) ||
        !box(&out->accent, x - 0.01f, top - 0.7f, zc - 0.3f, x + 0.01f,
             top - 0.5f, zc + 0.3f))
      return false;
  }
  return add_beacons(out, w * 1.6f, cab.top, cab.nose + 0.4f) &&
         add_beacons(out, w * 1.6f, top, L - 0.5f) &&
         add_tail_lights(out, w, floor + 0.3f, L);
}

static bool build_fire_engine(body_build_t* out, vec3_t h) {
  float w = h.x, L = h.z, floor = -h.y * 0.667f;
  cab_t cab = {w, floor, h.y * 0.62f, -L, 2.2f};
  out->own_paint = true;
  out->paint = vec3(0.72f, 0.05f, 0.04f);
  out->accent_colour = vec3(0.78f, 0.8f, 0.83f);
  out->extra_axles = 1;
  out->extra_axle_z = 1.4f;
  if (!add_cab(out, &cab) || !add_frame(out, w, floor, -L + 0.3f, L - 0.2f) ||
      !box(&out->body, -w, floor + 0.05f, -L + 2.35f, w, h.y * 0.5f, L))
    return false;
  /* Roller shutters on the flanks, a ladder along the roof. */
  for (int side = -1; side <= 1; side += 2)
    if (!box(&out->accent, side * w - 0.02f + (side > 0 ? 0.03f : -0.03f),
             floor + 0.25f, -L + 3.0f, side * w + (side > 0 ? 0.04f : -0.04f),
             h.y * 0.35f, L - 0.8f) ||
        !box(&out->accent, side * w * 0.45f - 0.04f, h.y * 0.5f, -L + 2.6f,
             side * w * 0.45f + 0.04f, h.y * 0.5f + 0.1f, L - 0.2f))
      return false;
  for (float z = -L + 2.8f; z < L - 0.3f; z += 0.45f)
    if (!box(&out->accent, -w * 0.45f, h.y * 0.5f + 0.02f, z, w * 0.45f,
             h.y * 0.5f + 0.07f, z + 0.04f))
      return false;
  return add_beacons(out, w * 1.5f, cab.top, cab.nose + 0.5f) &&
         add_tail_lights(out, w, floor + 0.3f, L);
}

bool vehicle_body_build(body_style_t style, vec3_t half, body_build_t* out) {
  switch (style) {
    case BODY_BUS:
      return build_bus(out, half);
    case BODY_BOX_TRUCK:
      return build_box_truck(out, half);
    case BODY_TANKER:
      return build_tanker(out, half);
    case BODY_FLATBED:
      return build_flatbed(out, half);
    case BODY_REFUSE:
      return build_refuse(out, half);
    case BODY_PICKUP:
      return build_pickup(out, half);
    case BODY_MIXER:
      return build_mixer(out, half);
    case BODY_AMBULANCE:
      return build_ambulance(out, half);
    case BODY_FIRE_ENGINE:
      return build_fire_engine(out, half);
    default:
      return false;
  }
}

bool vehicle_body_add_police(body_build_t* out, vec3_t half, float floor,
                             float roof, float roof_front, float roof_back) {
  float w = half.x, L = half.z, hy = half.y;
  out->own_paint = true;
  out->paint = vec3(0.04f, 0.09f, 0.3f);
  out->accent_colour = vec3(0.92f, 0.92f, 0.92f);
  float mid = (roof_front + roof_back) * 0.5f;
  for (int side = -1; side <= 1; side += 2)
    if (!box(&out->accent, side * (w + 0.01f) - 0.01f, floor + 0.12f * hy,
             -L * 0.3f, side * (w + 0.01f) + 0.01f, floor + 0.85f * hy,
             L * 0.5f))
      return false;
  return box(&out->trim, -w * 0.5f, roof, mid - 0.12f, w * 0.5f, roof + 0.04f,
             mid + 0.12f) &&
         box(&out->beacon_a, -w * 0.5f, roof + 0.04f, mid - 0.12f, 0,
             roof + 0.13f, mid + 0.12f) &&
         box(&out->beacon_b, 0, roof + 0.04f, mid - 0.12f, w * 0.5f,
             roof + 0.13f, mid + 0.12f);
}
