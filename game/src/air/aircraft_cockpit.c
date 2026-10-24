#include "aircraft_cockpit.h"

#include <math.h>

#include "font5x7.h"
#include "mesh_data.h"

#define PI 3.14159265f

/* One flight deck. Everything is in the aircraft's own frame, nose to -Z. The
 * instruments are drawn on a panel whose face looks back at the pilot. */
typedef struct {
  vec3_t eye;
  float panel_z, panel_y, half_width; /* the instrument panel */
  float pillar, roof_y;               /* the windscreen frame */
  int style;                          /* 0 yoke, 1 stick */
  vec3_t stick, stick_two;            /* where each pivots; two for a pair */
  int sticks;
  float stick_reach; /* how far it is pushed at full lock */
  float gauge;       /* radius of an ordinary dial */
  int sets;          /* instruments for the captain, and a first officer */
  float set_x[2];    /* the middle of each set on the panel */
  float gear_x;      /* the landing-gear lights and lever */
  float max_kmh;     /* full scale of the airspeed dial */
  int throttles;     /* levers; none for a rotorcraft, which has a collective */
  vec3_t throttle_at;
  bool engine_bars; /* a bar for each engine, in the middle of the panel */
} deck_t;

static const deck_t decks[AIRCRAFT_KIND_COUNT] = {
    [AIRCRAFT_PLANE] = {.eye = {-0.35f, 1.95f, -0.2f},
                        .panel_z = -1.05f,
                        .panel_y = 1.68f,
                        .half_width = 0.8f,
                        .pillar = 0.6f,
                        .roof_y = 2.35f,
                        .style = 0,
                        .stick = {-0.35f, 1.2f, -0.7f},
                        .sticks = 1,
                        .stick_reach = 0.3f,
                        .gauge = 0.075f,
                        .sets = 1,
                        .set_x = {-0.38f, 0},
                        .gear_x = -0.02f,
                        .max_kmh = 260.0f,
                        .throttles = 1,
                        .throttle_at = {-0.75f, 1.1f, -0.55f}},
    [AIRCRAFT_HELICOPTER] = {.eye = {-0.4f, 2.1f, -0.85f},
                             .panel_z = -1.7f,
                             .panel_y = 1.83f,
                             .half_width = 0.85f,
                             .pillar = 0.85f,
                             .roof_y = 2.5f,
                             .style = 1,
                             .stick = {-0.4f, 1.2f, -1.2f},
                             .sticks = 1,
                             .stick_reach = 0.3f,
                             .gauge = 0.075f,
                             .sets = 1,
                             .set_x = {-0.4f, 0},
                             .gear_x = -0.04f,
                             .max_kmh = 260.0f,
                             .throttles = 0,
                             .throttle_at = {-1.0f, 1.2f, -1.2f}},
    [AIRCRAFT_AIRLINER] = {.eye = {-0.7f, 8.4f, -29.2f},
                           .panel_z = -30.05f,
                           .panel_y = 8.13f,
                           .half_width = 1.55f,
                           .pillar = 1.4f,
                           .roof_y = 9.2f,
                           .style = 0,
                           .stick = {-0.7f, 7.3f, -29.45f},
                           .stick_two = {0.7f, 7.3f, -29.45f},
                           .sticks = 2,
                           .stick_reach = 0.35f,
                           .gauge = 0.082f,
                           .sets = 2,
                           .set_x = {-0.75f, 0.75f},
                           .gear_x = 0.0f,
                           .max_kmh = 600.0f,
                           .throttles = 4,
                           .throttle_at = {0.0f, 7.5f, -29.6f},
                           .engine_bars = true},
    [AIRCRAFT_JET] = {.eye = {0.0f, 2.3f, -2.3f},
                      .panel_z = -3.15f,
                      .panel_y = 2.03f,
                      .half_width = 0.45f,
                      .pillar = 0.42f,
                      .roof_y = 2.7f,
                      .style = 1,
                      .stick = {0.0f, 1.4f, -2.75f},
                      .sticks = 1,
                      .stick_reach = 0.3f,
                      .gauge = 0.062f,
                      .sets = 1,
                      .set_x = {-0.1f, 0},
                      .gear_x = 0.23f,
                      .max_kmh = 1000.0f,
                      .throttles = 1,
                      .throttle_at = {-0.55f, 1.6f, -2.5f}},
    [AIRCRAFT_GUNSHIP] = {.eye = {-0.55f, 3.0f, -5.3f},
                          .panel_z = -6.15f,
                          .panel_y = 2.73f,
                          .half_width = 0.95f,
                          .pillar = 0.95f,
                          .roof_y = 3.5f,
                          .style = 1,
                          .stick = {-0.55f, 2.0f, -5.9f},
                          .sticks = 1,
                          .stick_reach = 0.3f,
                          .gauge = 0.075f,
                          .sets = 1,
                          .set_x = {-0.55f, 0},
                          .gear_x = -0.2f,
                          .max_kmh = 350.0f,
                          .throttles = 0,
                          .throttle_at = {-1.2f, 2.0f, -5.8f}},
};

/* Where the face of the panel is, a little in front of its body. */
#define PANEL_HALF_HEIGHT 0.19f
#define PANEL_FACE 0.205f

vec3_t aircraft_cockpit_eye(aircraft_kind_t kind) { return decks[kind].eye; }

/* ---- shapes ---- */

static vertex_t flat_vertex(float x, float y, float z) {
  vertex_t v = {vec3(x, y, z), vec3(0, 0, 1), x, y};
  return v;
}

/* A disc of `radius` facing +Z at depth z. */
static void add_disc(mesh_data_t* m, float radius, float z, int segments) {
  uint32_t base = (uint32_t)m->vertex_count;
  mesh_data_add_vertex(m, flat_vertex(0, 0, z));
  for (int i = 0; i < segments; i++) {
    float a = 2.0f * PI * (float)i / (float)segments;
    mesh_data_add_vertex(m, flat_vertex(radius * cosf(a), radius * sinf(a), z));
  }
  for (int i = 0; i < segments; i++)
    mesh_data_add_triangle(m, base, base + 1 + (uint32_t)i,
                           base + 1 + (uint32_t)((i + 1) % segments));
}

/* A tick at `angle` (radians clockwise from the top) between two radii. */
static void add_tick(mesh_data_t* m, float angle, float r_in, float r_out,
                     float half_width, float z) {
  float dx = sinf(angle), dy = cosf(angle);
  float px = dy, py = -dx; /* to the right of the way it points */
  mesh_data_add_quad(
      m, vec3(dx * r_in - px * half_width, dy * r_in - py * half_width, z),
      vec3(dx * r_in + px * half_width, dy * r_in + py * half_width, z),
      vec3(dx * r_out + px * half_width, dy * r_out + py * half_width, z),
      vec3(dx * r_out - px * half_width, dy * r_out - py * half_width, z), 1);
}

/* A pointer along +Y, from a little behind the middle. */
static void add_needle(mesh_data_t* m, float length, float half_width,
                       float z) {
  mesh_data_add_quad(m, vec3(-half_width, -0.18f, z),
                     vec3(half_width, -0.18f, z),
                     vec3(half_width * 0.25f, length, z),
                     vec3(-half_width * 0.25f, length, z), 1);
}

/* A letter made of its pixels, `height` tall, its middle at `radius` from the
 * origin in the direction `angle` clockwise from the top, standing upright
 * about the origin like the figures round a compass card. */
static void add_glyph(mesh_data_t* m, char c, float cx, float cy, float height,
                      float rotation, float z) {
  float pixel = height / (float)FONT5X7_HEIGHT;
  float width = pixel * (float)FONT5X7_WIDTH;
  float sine = sinf(rotation), cosine = cosf(rotation);
  for (int row = 0; row < FONT5X7_HEIGHT; row++)
    for (int column = 0; column < FONT5X7_WIDTH; column++) {
      if (!font5x7_pixel(c, row, column)) continue;
      float x0 = -width * 0.5f + (float)column * pixel;
      float y1 = height * 0.5f - (float)row * pixel;
      float xs[4] = {x0, x0 + pixel, x0 + pixel, x0};
      float ys[4] = {y1 - pixel, y1 - pixel, y1, y1};
      vec3_t p[4];
      for (int i = 0; i < 4; i++)
        p[i] = vec3(cx + xs[i] * cosine + ys[i] * sine,
                    cy - xs[i] * sine + ys[i] * cosine, z);
      mesh_data_add_quad(m, p[0], p[1], p[2], p[3], 1);
    }
}

/* A line of text, left to right, `height` tall, centred on (cx, cy). */
static void add_text(mesh_data_t* m, const char* text, float cx, float cy,
                     float height, float z) {
  int n = 0;
  while (text[n]) n++;
  float advance = height * 6.0f / 7.0f;
  for (int i = 0; i < n; i++)
    add_glyph(m, text[i], cx + ((float)i - (float)(n - 1) * 0.5f) * advance, cy,
              height, 0.0f, z);
}

/* The upper (sign 1) or lower (-1) half of a sphere of radius 1, the rim in
 * the XZ plane. */
static void add_hemisphere(mesh_data_t* m, float sign) {
  const int rings = 8, segments = 24;
  for (int i = 0; i < rings; i++) {
    float f0 = sign * (PI * 0.5f) * (float)i / (float)rings;
    float f1 = sign * (PI * 0.5f) * (float)(i + 1) / (float)rings;
    for (int j = 0; j < segments; j++) {
      float t0 = 2.0f * PI * (float)j / (float)segments;
      float t1 = 2.0f * PI * (float)(j + 1) / (float)segments;
      vec3_t a = vec3(cosf(f0) * cosf(t0), sinf(f0), cosf(f0) * sinf(t0));
      vec3_t b = vec3(cosf(f1) * cosf(t0), sinf(f1), cosf(f1) * sinf(t0));
      vec3_t c = vec3(cosf(f1) * cosf(t1), sinf(f1), cosf(f1) * sinf(t1));
      vec3_t d = vec3(cosf(f0) * cosf(t1), sinf(f0), cosf(f0) * sinf(t1));
      if (sign > 0)
        mesh_data_add_quad(m, a, b, c, d, 1);
      else
        mesh_data_add_quad(m, d, c, b, a, 1);
    }
  }
}

static void build_instruments(aircraft_visual_t* v, renderer_t* renderer);

/* ---- one deck's fixed parts ---- */

static void build_deck(aircraft_kind_t kind, const deck_t* d, mesh_data_t* dark,
                       mesh_data_t* lit, mesh_data_t* stick) {
  float hw = d->half_width;
  /* The panel, its glare shield, and a console down each side. */
  mesh_data_add_box(dark, vec3(0, d->panel_y, d->panel_z),
                    vec3(hw, PANEL_HALF_HEIGHT, 0.2f));
  mesh_data_add_box(
      dark, vec3(0, d->panel_y + PANEL_HALF_HEIGHT + 0.03f, d->panel_z + 0.1f),
      vec3(hw, 0.02f, 0.16f));
  for (int side = -1; side <= 1; side += 2)
    mesh_data_add_box(
        dark, vec3(side * (hw + 0.12f), d->panel_y - 0.28f, d->eye.z - 0.3f),
        vec3(0.12f, 0.3f, 0.9f));
  /* The windscreen frame: a post each side, and the bar across the top. */
  for (int side = -1; side <= 1; side += 2)
    mesh_data_add_box(dark,
                      vec3(side * d->pillar, (d->panel_y + d->roof_y) * 0.5f,
                           d->panel_z - 0.05f),
                      vec3(0.025f, (d->roof_y - d->panel_y) * 0.5f, 0.025f));
  mesh_data_add_box(dark, vec3(0, d->roof_y, d->panel_z - 0.05f),
                    vec3(d->pillar, 0.025f, 0.03f));
  if (kind == AIRCRAFT_AIRLINER) {
    /* Six panes across the flight deck's windscreen, a sill, and the
     * pedestal between the two seats with the throttles on it. */
    static const float mullions[5] = {-0.9f, -0.45f, 0.0f, 0.45f, 0.9f};
    for (int i = 0; i < 5; i++)
      mesh_data_add_box(dark,
                        vec3(mullions[i], (d->panel_y + d->roof_y) * 0.5f,
                             d->panel_z - 0.05f),
                        vec3(0.015f, (d->roof_y - d->panel_y) * 0.5f, 0.02f));
    mesh_data_add_box(dark, vec3(0, d->panel_y - 0.3f, d->panel_z + 0.55f),
                      vec3(0.2f, 0.12f, 0.45f));
    /* The overhead: a row of switch panels along the roof. */
    mesh_data_add_box(dark, vec3(0, d->roof_y - 0.05f, d->panel_z + 0.7f),
                      vec3(0.9f, 0.04f, 0.5f));
    /* Seat backs of the two pilots, and a headrest to each. */
    for (int side = -1; side <= 1; side += 2) {
      mesh_data_add_box(dark,
                        vec3(side * 0.7f, d->eye.y - 0.5f, d->eye.z + 0.5f),
                        vec3(0.25f, 0.5f, 0.08f));
      mesh_data_add_box(dark,
                        vec3(side * 0.7f, d->eye.y + 0.12f, d->eye.z + 0.5f),
                        vec3(0.12f, 0.1f, 0.06f));
    }
    /* Lit: the autopilot strip along the glare shield, and its lamps. */
    mesh_data_add_box(
        lit,
        vec3(0, d->panel_y + PANEL_HALF_HEIGHT + 0.052f, d->panel_z + 0.12f),
        vec3(0.55f, 0.004f, 0.03f));
    for (int i = 0; i < 12; i++)
      mesh_data_add_box(
          lit,
          vec3(-0.5f + 0.09f * (float)i, d->roof_y - 0.088f, d->panel_z + 0.7f),
          vec3(0.02f, 0.004f, 0.02f));
    add_text(lit, "ENG", 0.0f, d->panel_y + 0.115f, 0.03f,
             d->panel_z + PANEL_FACE);
  }
  if (kind == AIRCRAFT_JET)
    mesh_data_add_box(
        lit,
        vec3(0, d->panel_y + PANEL_HALF_HEIGHT + 0.052f, d->panel_z + 0.12f),
        vec3(0.3f, 0.004f, 0.02f));
  /* A label over the landing-gear lights, on the aeroplanes that have any. */
  if (kind == AIRCRAFT_PLANE || kind == AIRCRAFT_AIRLINER ||
      kind == AIRCRAFT_JET)
    add_text(lit, "GEAR", d->gear_x, d->panel_y + 0.115f, 0.026f,
             d->panel_z + PANEL_FACE);
  /* The yoke's column and wheel, or a stick with a grip. */
  if (d->style == 0) {
    mesh_data_add_box(stick, vec3(0, 0, -0.12f), vec3(0.02f, 0.02f, 0.28f));
    mesh_data_add_box(stick, vec3(0, 0.08f, 0.2f), vec3(0.2f, 0.02f, 0.02f));
    mesh_data_add_box(stick, vec3(-0.19f, 0.0f, 0.2f),
                      vec3(0.02f, 0.1f, 0.02f));
    mesh_data_add_box(stick, vec3(0.19f, 0.0f, 0.2f), vec3(0.02f, 0.1f, 0.02f));
  } else {
    mesh_data_add_box(stick, vec3(0, 0.22f, 0), vec3(0.02f, 0.22f, 0.02f));
    mesh_data_add_ellipsoid(stick, vec3(0, 0.46f, 0), vec3(0.04f, 0.07f, 0.04f),
                            6, 8);
  }
}

static mesh_handle_t upload(renderer_t* renderer, mesh_data_t* mesh) {
  mesh_handle_t handle = renderer_upload_mesh(renderer, mesh);
  mesh_data_free(mesh);
  return handle;
}

/* ---- the instruments, shared by every deck, each of radius 1 ---- */

static void build_instruments(aircraft_visual_t* v, renderer_t* renderer) {
  mesh_data_t m = mesh_data_create();
  add_disc(&m, 1.12f, 0.0f, 28);
  v->gauge_body = upload(renderer, &m);
  /* Airspeed: eleven ticks over three quarters of a turn, and a label. */
  m = mesh_data_create();
  for (int i = 0; i <= 10; i++) {
    float angle = (-135.0f + 27.0f * (float)i) * PI / 180.0f;
    add_tick(&m, angle, i % 2 ? 0.78f : 0.68f, 0.96f, i % 2 ? 0.03f : 0.05f,
             0.05f);
  }
  add_text(&m, "KMH", 0.0f, -0.42f, 0.2f, 0.05f);
  v->gauge_ticks_speed = upload(renderer, &m);
  /* Altitude: ten ticks round the whole dial. */
  m = mesh_data_create();
  for (int i = 0; i < 10; i++)
    add_tick(&m, 2.0f * PI * (float)i / 10.0f, 0.7f, 0.96f, 0.05f, 0.05f);
  add_text(&m, "ALT", 0.0f, -0.42f, 0.2f, 0.05f);
  v->gauge_ticks_altitude = upload(renderer, &m);
  m = mesh_data_create();
  add_needle(&m, 0.92f, 0.05f, 0.1f);
  v->gauge_needle = upload(renderer, &m);
  m = mesh_data_create();
  add_needle(&m, 0.55f, 0.08f, 0.14f);
  v->gauge_needle_short = upload(renderer, &m);
  /* The compass card: a tick every ten degrees, long at every thirty, and
   * the letters round it, each standing out from the middle. */
  m = mesh_data_create();
  for (int i = 0; i < 36; i++) {
    float angle = 2.0f * PI * (float)i / 36.0f;
    bool cardinal = i % 9 == 0;
    if (!cardinal)
      add_tick(&m, angle, i % 3 ? 0.84f : 0.78f, 0.97f, i % 3 ? 0.012f : 0.03f,
               0.05f);
  }
  static const struct {
    char letter;
    int index;
  } cardinals[3] = {{'E', 9}, {'S', 18}, {'W', 27}};
  for (int i = 0; i < 3; i++) {
    float angle = 2.0f * PI * (float)cardinals[i].index / 36.0f;
    add_glyph(&m, cardinals[i].letter, sinf(angle) * 0.72f, cosf(angle) * 0.72f,
              0.3f, angle, 0.05f);
  }
  v->compass_card = upload(renderer, &m);
  m = mesh_data_create();
  add_glyph(&m, 'N', 0.0f, 0.72f, 0.3f, 0.0f, 0.06f);
  v->compass_north = upload(renderer, &m);
  /* The fixed mark the heading is read against. */
  m = mesh_data_create();
  {
    uint32_t base = (uint32_t)m.vertex_count;
    mesh_data_add_vertex(&m, flat_vertex(0.0f, 0.98f, 0.12f));
    mesh_data_add_vertex(&m, flat_vertex(0.08f, 1.14f, 0.12f));
    mesh_data_add_vertex(&m, flat_vertex(-0.08f, 1.14f, 0.12f));
    mesh_data_add_triangle(&m, base, base + 1, base + 2);
  }
  v->lubber = upload(renderer, &m);
  /* The horizon ball: a sphere, blue above, brown below. */
  m = mesh_data_create();
  add_hemisphere(&m, 1.0f);
  v->ball_sky = upload(renderer, &m);
  m = mesh_data_create();
  add_hemisphere(&m, -1.0f);
  v->ball_ground = upload(renderer, &m);
  /* Over it, the small aeroplane the horizon is read against. */
  m = mesh_data_create();
  mesh_data_add_box(&m, vec3(0, 0, 0.56f), vec3(0.55f, 0.04f, 0.01f));
  mesh_data_add_box(&m, vec3(0, 0.06f, 0.56f), vec3(0.04f, 0.1f, 0.01f));
  v->ball_symbol = upload(renderer, &m);
  m = mesh_data_create();
  add_disc(&m, 1.0f, 0.0f, 12);
  v->light_disc = upload(renderer, &m);
  m = mesh_data_create();
  mesh_data_add_box(&m, vec3(0, 0.07f, 0), vec3(0.014f, 0.07f, 0.014f));
  mesh_data_add_ellipsoid(&m, vec3(0, 0.15f, 0), vec3(0.03f, 0.03f, 0.03f), 5,
                          8);
  v->lever = upload(renderer, &m);
  m = mesh_data_create();
  mesh_data_add_box(&m, vec3(0, 0, 0), vec3(1, 1, 1));
  v->bar = upload(renderer, &m);
}

void aircraft_cockpit_create(aircraft_visual_t* v, renderer_t* renderer) {
  for (int k = 0; k < AIRCRAFT_KIND_COUNT; k++) {
    mesh_data_t dark = mesh_data_create(), lit = mesh_data_create(),
                stick = mesh_data_create();
    build_deck((aircraft_kind_t)k, &decks[k], &dark, &lit, &stick);
    v->cockpit_dark[k] = upload(renderer, &dark);
    v->cockpit_lit[k] = upload(renderer, &lit);
    v->cockpit_stick[k] = upload(renderer, &stick);
  }
  build_instruments(v, renderer);
}

void aircraft_cockpit_destroy(aircraft_visual_t* v, renderer_t* renderer) {
  for (int k = 0; k < AIRCRAFT_KIND_COUNT; k++) {
    renderer_release_mesh(renderer, v->cockpit_dark[k]);
    renderer_release_mesh(renderer, v->cockpit_lit[k]);
    renderer_release_mesh(renderer, v->cockpit_stick[k]);
  }
  mesh_handle_t all[] = {v->gauge_body,
                         v->gauge_ticks_speed,
                         v->gauge_ticks_altitude,
                         v->gauge_needle,
                         v->gauge_needle_short,
                         v->compass_card,
                         v->compass_north,
                         v->lubber,
                         v->ball_sky,
                         v->ball_ground,
                         v->ball_symbol,
                         v->light_disc,
                         v->lever,
                         v->bar};
  for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); i++)
    renderer_release_mesh(renderer, all[i]);
}

/* ---- drawing ---- */

/* Turned about Z, clockwise by `angle` as the pilot sees it. */
static mat4_t clockwise(float angle) {
  return mat4_from_quat(quat_from_axis_angle(vec3(0, 0, 1), -angle));
}

/* One dial: where it is on the panel (in units of the dial radius from
 * `centre`) and how big it is, as a transform. */
static mat4_t place(mat4_t model, vec3_t centre, float g, float dx, float dy,
                    float size) {
  return mat4_multiply(
      model, mat4_multiply(mat4_translation(vec3(centre.x + dx * g,
                                                 centre.y + dy * g, centre.z)),
                           mat4_scaling(vec3(g * size, g * size, g * size))));
}

typedef struct {
  material_t dark, light, needle, red, yellow, green, amber, sky, ground, grip,
      glow;
} paints_t;

static paints_t paints(void) {
  paints_t p;
  p.dark = material_rough(vec3(0.05f, 0.055f, 0.06f), 0.7f);
  p.light = material_plain(vec3(0.92f, 0.92f, 0.88f));
  p.light.emissive = 0.55f;
  p.needle = material_plain(vec3(1.0f, 0.62f, 0.12f));
  p.needle.emissive = 0.8f;
  p.red = material_plain(vec3(0.95f, 0.1f, 0.08f));
  p.red.emissive = 1.2f;
  p.yellow = material_plain(vec3(1.0f, 0.85f, 0.1f));
  p.yellow.emissive = 1.0f;
  p.green = material_plain(vec3(0.2f, 0.95f, 0.35f));
  p.green.emissive = 1.5f;
  p.amber = material_plain(vec3(1.0f, 0.55f, 0.08f));
  p.amber.emissive = 1.3f;
  p.sky = material_plain(vec3(0.2f, 0.45f, 0.95f));
  p.sky.emissive = 0.7f;
  p.ground = material_plain(vec3(0.45f, 0.28f, 0.12f));
  p.ground.emissive = 0.7f;
  p.grip = material_rough(vec3(0.12f, 0.12f, 0.13f), 0.5f);
  p.glow = material_plain(vec3(0.2f, 0.85f, 0.4f));
  p.glow.emissive = 1.0f;
  return p;
}

/* The speed, horizon, altitude and compass for one pilot. */
static void draw_set(const aircraft_visual_t* v, renderer_t* r, mat4_t model,
                     const deck_t* d, const aircraft_t* a, const paints_t* p,
                     float x, float height) {
  float g = d->gauge;
  vec3_t face = vec3(x, d->panel_y, d->panel_z + PANEL_FACE);
  /* Airspeed. */
  mat4_t at = place(model, face, g, -2.4f, 0.9f, 1.0f);
  renderer_draw_mesh(r, v->gauge_body, p->dark, at);
  renderer_draw_mesh(r, v->gauge_ticks_speed, p->light, at);
  float kmh = vec3_length(aircraft_velocity(a)) * 3.6f;
  float fraction = fminf(1.0f, kmh / d->max_kmh);
  renderer_draw_mesh(r, v->gauge_needle, p->needle,
                     mat4_multiply(at, clockwise((-135.0f + 270.0f * fraction) *
                                                 PI / 180.0f)));
  /* Altitude over the ground: the long needle once round for every thousand
   * metres, the short one once round for ten thousand. */
  at = place(model, face, g, 2.4f, 0.9f, 1.0f);
  renderer_draw_mesh(r, v->gauge_body, p->dark, at);
  renderer_draw_mesh(r, v->gauge_ticks_altitude, p->light, at);
  renderer_draw_mesh(
      r, v->gauge_needle_short, p->light,
      mat4_multiply(at, clockwise(2.0f * PI * height / 10000.0f)));
  renderer_draw_mesh(
      r, v->gauge_needle, p->needle,
      mat4_multiply(at,
                    clockwise(2.0f * PI * fmodf(height, 1000.0f) / 1000.0f)));
  /* The horizon: a ball turned against the aircraft's own attitude, so the
   * sky is above the real horizon whatever the nose and wings do. */
  at = place(model, face, g, 0.0f, 0.9f, 1.3f);
  renderer_draw_mesh(r, v->gauge_body, p->dark, at);
  quat_t attitude =
      quat_multiply(quat_from_axis_angle(vec3(1, 0, 0), a->pitch),
                    quat_from_axis_angle(vec3(0, 0, -1), a->roll));
  mat4_t ball = mat4_multiply(
      at, mat4_multiply(mat4_translation(vec3(0, 0, -0.5f)),
                        mat4_from_quat(quat_conjugate(attitude))));
  renderer_draw_mesh(r, v->ball_sky, p->sky, ball);
  renderer_draw_mesh(r, v->ball_ground, p->ground, ball);
  renderer_draw_mesh(r, v->ball_symbol, p->yellow, at);
  /* The compass: the card turns the other way to the nose. The bearing is
   * clockwise from north, and the nose at yaw 0 points north (-Z). */
  at = place(model, face, g, -1.3f, -1.1f, 0.95f);
  renderer_draw_mesh(r, v->gauge_body, p->dark, at);
  mat4_t card = mat4_multiply(
      at, mat4_from_quat(quat_from_axis_angle(vec3(0, 0, 1), -a->yaw)));
  renderer_draw_mesh(r, v->compass_card, p->light, card);
  renderer_draw_mesh(r, v->compass_north, p->red, card);
  renderer_draw_mesh(r, v->lubber, p->yellow, at);
}

/* The lights and the lever for the wheels: green when they are down and
 * locked, red while they move, dark when they are up. */
static void draw_gear_panel(const aircraft_visual_t* v, renderer_t* r,
                            mat4_t model, const deck_t* d, const aircraft_t* a,
                            const paints_t* p) {
  float g = d->gauge;
  vec3_t face = vec3(d->gear_x, d->panel_y, d->panel_z + PANEL_FACE);
  bool moving = a->gear > 0.01f && a->gear < 0.99f;
  bool down = a->gear >= 0.99f;
  const material_t* lamp = moving ? &p->red : (down ? &p->green : &p->dark);
  static const float lights[3][2] = {
      {-0.75f, 0.1f}, {0.0f, 0.1f}, {0.75f, 0.1f}};
  for (int i = 0; i < 3; i++)
    renderer_draw_mesh(
        r, v->light_disc, *lamp,
        place(model, face, g, lights[i][0], lights[i][1] - 0.2f, 0.34f));
  /* The lever: up when the wheels are, down when they are down. */
  float travel = a->gear_down ? -0.5f : 0.5f;
  renderer_draw_mesh(r, v->bar, p->dark,
                     place(model, vec3(face.x, face.y, face.z + 0.002f), g,
                           0.0f, -1.5f, 1.0f));
  mat4_t slot = place(model, face, g, 0.0f, -1.5f + travel, 0.5f);
  renderer_draw_mesh(r, v->bar, p->light,
                     mat4_multiply(slot, mat4_scaling(vec3(0.5f, 0.2f, 0.1f))));
}

void aircraft_visual_draw_cockpit(const aircraft_visual_t* v,
                                  renderer_t* renderer, const aircraft_t* a,
                                  aircraft_controls_t stick, float height) {
  const deck_t* d = &decks[a->kind];
  mat4_t model = transform_to_matrix(aircraft_transform(a));
  paints_t p = paints();
  renderer_draw_mesh(renderer, v->cockpit_dark[a->kind], p.dark, model);
  renderer_draw_mesh(renderer, v->cockpit_lit[a->kind], p.glow, model);
  for (int i = 0; i < d->sets; i++)
    draw_set(v, renderer, model, d, a, &p, d->set_x[i], height);
  if (aircraft_is_fixed_wing(a->kind))
    draw_gear_panel(v, renderer, model, d, a, &p);
  /* The yoke or stick leans with the controls: back pulls the nose up. */
  for (int i = 0; i < d->sticks; i++) {
    vec3_t pivot = i == 0 ? d->stick : d->stick_two;
    mat4_t lean = mat4_multiply(
        mat4_translation(pivot),
        mat4_from_quat(quat_multiply(
            quat_from_axis_angle(vec3(1, 0, 0), stick.pitch * 0.45f),
            quat_from_axis_angle(vec3(0, 0, 1), -stick.roll * 0.45f))));
    renderer_draw_mesh(renderer, v->cockpit_stick[a->kind], p.grip,
                       mat4_multiply(model, lean));
  }
  /* The throttle levers lean forward with the power; a rotorcraft's single
   * lever is the collective, up to climb. */
  float power = aircraft_is_fixed_wing(a->kind)
                    ? a->power
                    : 0.5f + 0.5f * stick.power_change;
  int levers = d->throttles > 0 ? d->throttles : 1;
  for (int i = 0; i < levers; i++) {
    float spread = d->throttles > 1
                       ? ((float)i - (float)(levers - 1) * 0.5f) * 0.075f
                       : 0.0f;
    vec3_t at =
        vec3(d->throttle_at.x + spread, d->throttle_at.y, d->throttle_at.z);
    mat4_t lean = mat4_multiply(mat4_translation(at),
                                mat4_from_quat(quat_from_axis_angle(
                                    vec3(1, 0, 0), (0.5f - power) * 1.1f)));
    renderer_draw_mesh(renderer, v->lever, p.grip, mat4_multiply(model, lean));
  }
  /* A bar of light for each engine, rising with its power. */
  if (d->engine_bars) {
    for (int i = 0; i < 4; i++) {
      float x = -0.15f + 0.1f * (float)i;
      float h = 0.012f + 0.045f * a->power;
      mat4_t bar = mat4_multiply(
          model, mat4_multiply(mat4_translation(vec3(x, d->panel_y - 0.06f + h,
                                                     d->panel_z + PANEL_FACE)),
                               mat4_scaling(vec3(0.012f, h, 0.003f))));
      renderer_draw_mesh(renderer, v->bar, a->power > 0.85f ? p.amber : p.green,
                         bar);
    }
  }
}

/* A part hung at `local` and turned about an axis, as in the visuals. */
static mat4_t spun(transform_t base, vec3_t local, float angle) {
  transform_t part = base;
  part.position = transform_point(base, local);
  part.rotation =
      quat_multiply(base.rotation, quat_from_axis_angle(vec3(0, 1, 0), angle));
  return transform_to_matrix(part);
}

void aircraft_visual_draw_spinners(const aircraft_visual_t* v,
                                   renderer_t* renderer, const aircraft_t* a) {
  transform_t base = aircraft_transform(a);
  material_t trim = material_rough(vec3(0.1f, 0.1f, 0.11f), 0.6f);
  if (a->kind == AIRCRAFT_PLANE) {
    transform_t part = base;
    part.position = transform_point(base, vec3(0, 1.3f, -3.95f));
    part.rotation = quat_multiply(
        base.rotation, quat_from_axis_angle(vec3(0, 0, 1), a->rotor));
    renderer_draw_mesh(renderer, v->plane_prop, trim,
                       transform_to_matrix(part));
  } else if (a->kind == AIRCRAFT_HELICOPTER) {
    renderer_draw_mesh(renderer, v->heli_rotor, trim,
                       spun(base, vec3(0, 3.15f, 0.5f), a->rotor));
  } else if (a->kind == AIRCRAFT_GUNSHIP) {
    renderer_draw_mesh(renderer, v->gunship_rotor, trim,
                       spun(base, vec3(0, 4.75f, -4.0f), a->rotor));
    renderer_draw_mesh(renderer, v->gunship_rotor, trim,
                       spun(base, vec3(0, 5.95f, 5.6f), 0.785f - a->rotor));
  }
}

void aircraft_visual_draw_sight(renderer_t* renderer, const aircraft_t* a) {
  if (!aircraft_has_missiles(a->kind)) return;
  transform_t t = aircraft_transform(a);
  vec3_t green = vec3(0.3f, 1.0f, 0.4f);
  vec3_t eye = transform_point(t, aircraft_cockpit_eye(a->kind));
  vec3_t forward = quat_rotate(t.rotation, vec3(0, 0, -1));
  vec3_t right = quat_rotate(t.rotation, vec3(1, 0, 0));
  vec3_t up = quat_rotate(t.rotation, vec3(0, 1, 0));
  vec3_t centre = vec3_add(eye, vec3_scale(forward, 120.0f));
  /* A ring and a cross, 120 m out along the nose. */
  vec3_t previous = vec3_add(centre, vec3_scale(right, 3.0f));
  for (int i = 1; i <= 16; i++) {
    float angle = 6.2831853f * (float)i / 16.0f;
    vec3_t point =
        vec3_add(centre, vec3_add(vec3_scale(right, 3.0f * cosf(angle)),
                                  vec3_scale(up, 3.0f * sinf(angle))));
    renderer_draw_line(renderer, previous, point, green);
    previous = point;
  }
  renderer_draw_line(renderer, vec3_sub(centre, vec3_scale(right, 6.0f)),
                     vec3_sub(centre, vec3_scale(right, 3.6f)), green);
  renderer_draw_line(renderer, vec3_add(centre, vec3_scale(right, 3.6f)),
                     vec3_add(centre, vec3_scale(right, 6.0f)), green);
  renderer_draw_line(renderer, vec3_add(centre, vec3_scale(up, 3.6f)),
                     vec3_add(centre, vec3_scale(up, 6.0f)), green);
}
