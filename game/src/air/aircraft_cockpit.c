#include "aircraft_cockpit.h"

#include <math.h>

#include "mesh_data.h"

typedef struct {
  vec3_t eye;
  float panel_z, panel_y, half_width; /* the instrument panel */
  float pillar, roof_y;               /* the windscreen frame */
  int style;                          /* 0 yoke, 1 stick */
  vec3_t stick;                       /* where the yoke or stick pivots */
  float stick_reach;                  /* how far it is pushed at full lock */
} deck_t;

static const deck_t decks[AIRCRAFT_KIND_COUNT] = {
    [AIRCRAFT_PLANE] = {{-0.35f, 1.95f, -0.2f},
                        -1.2f,
                        1.5f,
                        0.8f,
                        0.6f,
                        2.35f,
                        0,
                        {-0.35f, 1.35f, -0.85f},
                        0.3f},
    [AIRCRAFT_HELICOPTER] = {{-0.4f, 2.1f, -0.85f},
                             -1.75f,
                             1.7f,
                             0.85f,
                             0.85f,
                             2.5f,
                             1,
                             {-0.4f, 1.2f, -1.2f},
                             0.3f},
    [AIRCRAFT_AIRLINER] = {{-0.7f, 8.4f, -29.2f},
                           -30.4f,
                           7.75f,
                           1.5f,
                           1.35f,
                           9.2f,
                           0,
                           {-0.7f, 7.4f, -29.6f},
                           0.35f},
    [AIRCRAFT_JET] = {{0.0f, 2.3f, -2.3f},
                      -3.55f,
                      1.95f,
                      0.42f,
                      0.42f,
                      2.7f,
                      1,
                      {0.0f, 1.4f, -2.9f},
                      0.3f},
    [AIRCRAFT_GUNSHIP] = {{-0.55f, 3.0f, -5.3f},
                          -6.8f,
                          2.4f,
                          0.95f,
                          0.95f,
                          3.5f,
                          1,
                          {-0.55f, 2.0f, -6.0f},
                          0.3f},
};

vec3_t aircraft_cockpit_eye(aircraft_kind_t kind) { return decks[kind].eye; }

static void build_deck(const deck_t* d, mesh_data_t* dark, mesh_data_t* lit,
                       mesh_data_t* stick) {
  float hw = d->half_width;
  /* The panel, its glare shield, and a console down each side. */
  mesh_data_add_box(dark, vec3(0, d->panel_y, d->panel_z),
                    vec3(hw, 0.11f, 0.2f));
  mesh_data_add_box(dark, vec3(0, d->panel_y + 0.14f, d->panel_z + 0.1f),
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
  /* Dials: two rows of lit faces across the front of the panel. */
  int columns = (int)(hw * 2.0f / 0.1f);
  if (columns < 3) columns = 3;
  for (int row = 0; row < 2; row++)
    for (int c = 0; c < columns; c++) {
      float x = -hw + ((float)c + 0.5f) * (hw * 2.0f / (float)columns);
      mesh_data_add_box(
          lit,
          vec3(x, d->panel_y + 0.04f - 0.07f * (float)row, d->panel_z + 0.205f),
          vec3(0.013f, 0.013f, 0.004f));
    }
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

/* A half-disc of `radius` below or above the horizontal. */
static void half_disc(mesh_data_t* m, float radius, float sign) {
  float r = radius, d = radius * 0.7071f;
  mesh_data_add_quad(m, vec3(-r, 0, 0), vec3(0, 0, 0), vec3(0, sign * r, 0),
                     vec3(-d, sign * d, 0), 1);
  mesh_data_add_quad(m, vec3(0, 0, 0), vec3(r, 0, 0), vec3(d, sign * d, 0),
                     vec3(0, sign * r, 0), 1);
}

static mesh_handle_t upload(renderer_t* renderer, mesh_data_t* mesh) {
  mesh_handle_t handle = renderer_upload_mesh(renderer, mesh);
  mesh_data_free(mesh);
  return handle;
}

void aircraft_cockpit_create(aircraft_visual_t* v, renderer_t* renderer) {
  for (int k = 0; k < AIRCRAFT_KIND_COUNT; k++) {
    mesh_data_t dark = mesh_data_create(), lit = mesh_data_create(),
                stick = mesh_data_create();
    build_deck(&decks[k], &dark, &lit, &stick);
    v->cockpit_dark[k] = upload(renderer, &dark);
    v->cockpit_lit[k] = upload(renderer, &lit);
    v->cockpit_stick[k] = upload(renderer, &stick);
  }
  mesh_data_t sky = mesh_data_create(), ground = mesh_data_create();
  half_disc(&sky, 0.05f, 1.0f);
  half_disc(&ground, 0.05f, -1.0f);
  v->horizon_sky = upload(renderer, &sky);
  v->horizon_ground = upload(renderer, &ground);
}

void aircraft_cockpit_destroy(aircraft_visual_t* v, renderer_t* renderer) {
  for (int k = 0; k < AIRCRAFT_KIND_COUNT; k++) {
    renderer_release_mesh(renderer, v->cockpit_dark[k]);
    renderer_release_mesh(renderer, v->cockpit_lit[k]);
    renderer_release_mesh(renderer, v->cockpit_stick[k]);
  }
  renderer_release_mesh(renderer, v->horizon_sky);
  renderer_release_mesh(renderer, v->horizon_ground);
}

void aircraft_visual_draw_cockpit(const aircraft_visual_t* v,
                                  renderer_t* renderer, const aircraft_t* a,
                                  aircraft_controls_t stick) {
  const deck_t* d = &decks[a->kind];
  mat4_t model = transform_to_matrix(aircraft_transform(a));
  material_t dark = material_rough(vec3(0.06f, 0.065f, 0.07f), 0.7f);
  renderer_draw_mesh(renderer, v->cockpit_dark[a->kind], dark, model);
  material_t glow = material_plain(vec3(0.2f, 0.85f, 0.4f));
  glow.emissive = 0.9f;
  renderer_draw_mesh(renderer, v->cockpit_lit[a->kind], glow, model);
  /* The yoke or stick leans with the controls: back pulls the nose up. */
  mat4_t lean = mat4_multiply(
      mat4_translation(d->stick),
      mat4_from_quat(quat_multiply(
          quat_from_axis_angle(vec3(1, 0, 0), stick.pitch * 0.45f),
          quat_from_axis_angle(vec3(0, 0, 1), -stick.roll * 0.45f))));
  material_t grip = material_rough(vec3(0.12f, 0.12f, 0.13f), 0.5f);
  renderer_draw_mesh(renderer, v->cockpit_stick[a->kind], grip,
                     mat4_multiply(model, lean));
  /* A roll indicator: the horizon tilts against the aircraft's own bank. */
  mat4_t at = mat4_multiply(
      model,
      mat4_multiply(
          mat4_translation(vec3(d->half_width * 0.45f, d->panel_y + 0.0f,
                                d->panel_z + 0.212f)),
          mat4_from_quat(quat_from_axis_angle(vec3(0, 0, 1), -a->roll))));
  material_t sky = material_plain(vec3(0.25f, 0.5f, 0.9f));
  sky.emissive = 0.8f;
  material_t ground = material_plain(vec3(0.5f, 0.32f, 0.15f));
  ground.emissive = 0.8f;
  renderer_draw_mesh(renderer, v->horizon_sky, sky, at);
  renderer_draw_mesh(renderer, v->horizon_ground, ground, at);
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
