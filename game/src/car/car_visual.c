#include "car_visual.h"

#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* The body is lofted through side-profile sections from nose to tail:
 * each section has a height range and a half width, and the quads
 * between consecutive sections make the bonnet, windscreen, roof, boot
 * and flanks. Dimensions scale with the chassis half extents. */
typedef struct {
  float z;          /* along the car, nose is -Z */
  float top;        /* y of the upper edge */
  float bottom;     /* y of the lower edge */
  float half_width; /* x */
} section_t;

static bool loft(mesh_data_t* mesh, const section_t* sections, int count) {
  for (int i = 0; i + 1 < count; i++) {
    section_t a = sections[i], b = sections[i + 1];
    vec3_t a_tl = vec3(-a.half_width, a.top, a.z),
           a_tr = vec3(a.half_width, a.top, a.z);
    vec3_t a_bl = vec3(-a.half_width, a.bottom, a.z),
           a_br = vec3(a.half_width, a.bottom, a.z);
    vec3_t b_tl = vec3(-b.half_width, b.top, b.z),
           b_tr = vec3(b.half_width, b.top, b.z);
    vec3_t b_bl = vec3(-b.half_width, b.bottom, b.z),
           b_br = vec3(b.half_width, b.bottom, b.z);
    /* Anticlockwise seen from outside: top faces up, sides face out,
     * bottom faces down. */
    if (!mesh_data_add_quad(mesh, a_tr, a_tl, b_tl, b_tr, 1) || /* top */
        !mesh_data_add_quad(mesh, a_bl, a_br, b_br, b_bl, 1) || /* bottom */
        !mesh_data_add_quad(mesh, a_br, a_tr, b_tr, b_br, 1) || /* right */
        !mesh_data_add_quad(mesh, a_tl, a_bl, b_bl, b_tl, 1))   /* left */
      return false;
  }
  section_t nose = sections[0], tail = sections[count - 1];
  return mesh_data_add_quad(mesh, vec3(-nose.half_width, nose.bottom, nose.z),
                            vec3(-nose.half_width, nose.top, nose.z),
                            vec3(nose.half_width, nose.top, nose.z),
                            vec3(nose.half_width, nose.bottom, nose.z), 1) &&
         mesh_data_add_quad(mesh, vec3(tail.half_width, tail.bottom, tail.z),
                            vec3(tail.half_width, tail.top, tail.z),
                            vec3(-tail.half_width, tail.top, tail.z),
                            vec3(-tail.half_width, tail.bottom, tail.z), 1);
}

static mesh_handle_t upload(renderer_t* renderer, mesh_data_t* mesh) {
  mesh_handle_t handle = renderer_upload_mesh(renderer, mesh);
  mesh_data_free(mesh);
  return handle;
}

/* Meshes a body style may leave empty upload as "none". */
static mesh_handle_t upload_optional(renderer_t* renderer, mesh_data_t* mesh) {
  mesh_handle_t none = {0};
  if (mesh->vertex_count == 0) {
    mesh_data_free(mesh);
    return none;
  }
  return upload(renderer, mesh);
}

static void draw_optional(renderer_t* renderer, mesh_handle_t mesh,
                          material_t material, mat4_t model) {
  if (mesh.id != 0) renderer_draw_mesh(renderer, mesh, material, model);
}

/* A flat ring in the xy plane about the origin, facing +z, between two
 * radii and `half_z` thick each way. */
static void add_ring(mesh_data_t* mesh, float outer, float inner, float half_z,
                     int segments) {
  for (int k = 0; k < segments; k++) {
    float a0 = 2.0f * (float)M_PI * (float)k / (float)segments;
    float a1 = 2.0f * (float)M_PI * (float)(k + 1) / (float)segments;
    float c0 = cosf(a0), s0 = sinf(a0), c1 = cosf(a1), s1 = sinf(a1);
    vec3_t io0 = vec3(inner * c0, inner * s0, half_z);
    vec3_t io1 = vec3(inner * c1, inner * s1, half_z);
    vec3_t oo0 = vec3(outer * c0, outer * s0, half_z);
    vec3_t oo1 = vec3(outer * c1, outer * s1, half_z);
    vec3_t ib0 = vec3(inner * c0, inner * s0, -half_z);
    vec3_t ib1 = vec3(inner * c1, inner * s1, -half_z);
    vec3_t ob0 = vec3(outer * c0, outer * s0, -half_z);
    vec3_t ob1 = vec3(outer * c1, outer * s1, -half_z);
    mesh_data_add_quad(mesh, io0, oo0, oo1, io1, 1); /* front */
    mesh_data_add_quad(mesh, ib1, ob1, ob0, ib0, 1); /* back */
    mesh_data_add_quad(mesh, ob0, ob1, oo1, oo0, 1); /* outer wall */
    mesh_data_add_quad(mesh, ib0, io0, io1, ib1, 1); /* inner wall */
  }
}

car_visual_t car_visual_create(renderer_t* renderer,
                               const vehicle_params_t* p) {
  car_visual_t visual;
  vec3_t h = p->chassis_half_extents;
  float w = h.x, L = h.z, floor = -h.y * 0.55f;
  /* Lower body: a shallow wedge rising from the nose to the tail lights. */
  section_t body[] = {
      {-L, floor + 0.28f * h.y, floor, w * 0.84f},
      {-L * 0.92f, floor + 0.62f * h.y, floor, w * 0.96f},
      {-L * 0.55f, floor + 0.78f * h.y, floor, w},
      {L * 0.6f, floor + 0.86f * h.y, floor, w},
      {L * 0.96f, floor + 0.9f * h.y, floor, w * 0.96f},
      {L, floor + 0.7f * h.y, floor, w * 0.86f},
  };
  mesh_data_t body_mesh = mesh_data_create();
  loft(&body_mesh, body, (int)(sizeof(body) / sizeof(body[0])));
  /* Cabin: windscreen, roof and rear window as a second loft set back. */
  float sill = floor + 0.86f * h.y;
  float roof = sill + 1.45f * h.y;
  section_t cabin[] = {
      {-L * 0.42f, sill + 0.02f, sill - 0.05f, w * 0.9f},
      {-L * 0.08f, roof, sill - 0.05f, w * 0.82f},
      {L * 0.42f, roof, sill - 0.05f, w * 0.82f},
      {L * 0.68f, sill + 0.1f, sill - 0.05f, w * 0.88f},
  };
  mesh_data_t cabin_mesh = mesh_data_create();
  loft(&cabin_mesh, cabin, (int)(sizeof(cabin) / sizeof(cabin[0])));
  /* Pillars and the roof are paint; the glass sits just outside the cabin
   * faces on the windscreen, rear window and flanks. */
  mesh_data_t glass_mesh = mesh_data_create();
  float gap = 0.015f;
  mesh_data_add_quad(&glass_mesh,
                     vec3(-w * 0.86f, sill + 0.08f, -L * 0.42f - gap),
                     vec3(-w * 0.78f, roof - 0.04f, -L * 0.08f - gap),
                     vec3(w * 0.78f, roof - 0.04f, -L * 0.08f - gap),
                     vec3(w * 0.86f, sill + 0.08f, -L * 0.42f - gap), 1);
  mesh_data_add_quad(&glass_mesh,
                     vec3(w * 0.84f, sill + 0.14f, L * 0.68f + gap),
                     vec3(w * 0.78f, roof - 0.04f, L * 0.42f + gap),
                     vec3(-w * 0.78f, roof - 0.04f, L * 0.42f + gap),
                     vec3(-w * 0.84f, sill + 0.14f, L * 0.68f + gap), 1);
  for (int side = -1; side <= 1; side += 2) {
    float x = side * (w * 0.82f + gap);
    vec3_t a = vec3(x, sill + 0.06f, -L * 0.22f),
           b = vec3(x, sill + 0.06f, L * 0.5f);
    vec3_t c = vec3(x, roof - 0.06f, L * 0.4f),
           d = vec3(x, roof - 0.06f, -L * 0.1f);
    if (side > 0)
      mesh_data_add_quad(&glass_mesh, b, a, d, c, 1);
    else
      mesh_data_add_quad(&glass_mesh, a, b, c, d, 1);
  }
  mesh_data_t lights_mesh = mesh_data_create();
  mesh_data_t rear_mesh = mesh_data_create();
  for (int side = -1; side <= 1; side += 2) {
    mesh_data_add_box(&lights_mesh,
                      vec3(side * w * 0.6f, floor + 0.45f * h.y, -L - 0.01f),
                      vec3(w * 0.16f, h.y * 0.12f, 0.02f));
    mesh_data_add_box(&rear_mesh,
                      vec3(side * w * 0.6f, floor + 0.5f * h.y, L + 0.01f),
                      vec3(w * 0.18f, h.y * 0.1f, 0.02f));
  }
  mesh_data_t trim_mesh = mesh_data_create();
  mesh_data_add_box(&trim_mesh, vec3(0, floor + 0.1f * h.y, -L),
                    vec3(w * 0.9f, h.y * 0.12f, 0.06f));
  mesh_data_add_box(&trim_mesh, vec3(0, floor + 0.1f * h.y, L),
                    vec3(w * 0.9f, h.y * 0.12f, 0.06f));

  /* Police cars are saloons with livery and a light bar; everything else
   * that is not a plain car replaces the saloon body with its own. */
  body_style_t style = body_style_from_name(p->body);
  body_build_t build;
  body_build_init(&build);
  if (style == BODY_POLICE) {
    vehicle_body_add_police(&build, h, floor, roof, -L * 0.08f, L * 0.42f);
  } else if (style != BODY_CAR && vehicle_body_build(style, h, &build)) {
    mesh_data_free(&body_mesh);
    mesh_data_free(&cabin_mesh);
    mesh_data_free(&glass_mesh);
    mesh_data_free(&lights_mesh);
    mesh_data_free(&rear_mesh);
    mesh_data_free(&trim_mesh);
    body_mesh = build.body, build.body = mesh_data_create();
    cabin_mesh = build.cabin, build.cabin = mesh_data_create();
    glass_mesh = build.glass, build.glass = mesh_data_create();
    lights_mesh = build.lights, build.lights = mesh_data_create();
    rear_mesh = build.rear, build.rear = mesh_data_create();
    trim_mesh = build.trim, build.trim = mesh_data_create();
  } else {
    style = BODY_CAR;
  }

  mesh_data_t wheel = mesh_data_create();
  mesh_data_add_cylinder_x(&wheel, vec3_zero(), p->wheel_radius, p->wheel_width,
                           24);
  mesh_data_add_box(&wheel, vec3_zero(),
                    vec3(p->wheel_width * 0.55f, p->wheel_radius * 0.12f,
                         p->wheel_radius * 0.62f));
  mesh_data_add_box(&wheel, vec3_zero(),
                    vec3(p->wheel_width * 0.55f, p->wheel_radius * 0.62f,
                         p->wheel_radius * 0.12f));

  float rest_hub_y = p->wheel_attach_height - p->suspension_rest_length * 0.65f;
  for (int i = 0; i < 4; i++)
    visual.wheel_centres[i] =
        vec3((i % 2 == 0 ? -1.0f : 1.0f) * p->track_width * 0.5f, rest_hub_y,
             (i < 2 ? -1.0f : 1.0f) * p->wheelbase * 0.5f);
  visual.wheel_radius = p->wheel_radius;
  visual.ride_height = p->wheel_radius - rest_hub_y;
  visual.half_extents = h;
  /* The inside, from the seat: a dashboard across the base of the
   * windscreen with a hooded instrument binnacle, and a steering wheel: a
   * ring of twelve short bars, a hub and three spokes, centred on its own
   * origin with its axis along +z. */
  mesh_data_t dash_mesh = mesh_data_create();
  float dash_z = -L * 0.3f, dash_y = sill + 0.07f;
  /* The dash top, a hooded binnacle before the driver and a glovebox face. */
  mesh_data_add_box(&dash_mesh, vec3(0, dash_y - 0.04f, dash_z - 0.2f),
                    vec3(w * 0.92f, 0.05f, 0.3f));
  mesh_data_add_box(&dash_mesh,
                    vec3(-w * 0.42f, dash_y + 0.035f, dash_z - 0.06f),
                    vec3(0.2f, 0.045f, 0.09f));
  mesh_data_add_box(&dash_mesh, vec3(0, dash_y - 0.17f, dash_z + 0.1f),
                    vec3(w * 0.92f, 0.12f, 0.04f));
  mesh_data_t rim_mesh = mesh_data_create();
  add_ring(&rim_mesh, 0.2f, 0.16f, 0.017f, 28);
  mesh_data_t hub_mesh = mesh_data_create();
  mesh_data_add_box(&hub_mesh, vec3(0, 0, 0), vec3(0.05f, 0.05f, 0.03f));
  mesh_data_add_box(&hub_mesh, vec3(0, -0.085f, 0),
                    vec3(0.016f, 0.085f, 0.016f));
  mesh_data_add_box(&hub_mesh, vec3(-0.085f, 0, 0),
                    vec3(0.085f, 0.016f, 0.016f));
  mesh_data_add_box(&hub_mesh, vec3(0.085f, 0, 0),
                    vec3(0.085f, 0.016f, 0.016f));
  mesh_data_t smoke_mesh = mesh_data_create();
  mesh_data_add_ellipsoid(&smoke_mesh, vec3_zero(), vec3(0.5f, 0.45f, 0.5f), 3,
                          6);
  visual.smoke_puff = upload(renderer, &smoke_mesh);
  visual.dashboard = upload(renderer, &dash_mesh);
  visual.wheel_rim = upload(renderer, &rim_mesh);
  visual.wheel_hub = upload(renderer, &hub_mesh);
  visual.dash_material = material_rough(vec3(0.1f, 0.1f, 0.11f), 0.8f);
  visual.chassis = upload_optional(renderer, &body_mesh);
  visual.cabin = upload_optional(renderer, &cabin_mesh);
  visual.glass = upload_optional(renderer, &glass_mesh);
  visual.lights = upload_optional(renderer, &lights_mesh);
  visual.rear_lights = upload_optional(renderer, &rear_mesh);
  visual.trim = upload_optional(renderer, &trim_mesh);
  visual.style = style;
  visual.own_paint = build.own_paint;
  visual.own_colour = build.paint;
  visual.emergency =
      build.beacon_a.vertex_count > 0 || build.beacon_b.vertex_count > 0;
  visual.accent = upload_optional(renderer, &build.accent);
  visual.beacon_a = upload_optional(renderer, &build.beacon_a);
  visual.beacon_b = upload_optional(renderer, &build.beacon_b);
  visual.accent_material = material_rough(build.accent_colour, 0.55f);
  visual.beacon_a_material = material_rough(build.beacon_a_colour, 0.3f);
  visual.beacon_b_material = material_rough(build.beacon_b_colour, 0.3f);
  visual.extra_wheel_count = 0;
  for (int axle = 0; axle < build.extra_axles && axle < 2; axle++)
    for (int side = -1; side <= 1; side += 2)
      visual.extra_wheel_centres[visual.extra_wheel_count++] =
          vec3(side * p->track_width * 0.5f, rest_hub_y,
               p->wheelbase * 0.5f + build.extra_axle_z * (float)(axle + 1));
  body_build_free(&build);
  visual.wheel = upload(renderer, &wheel);
  visual.paint = material_plain(vec3(0.72f, 0.1f, 0.08f));
  if (visual.own_paint) visual.paint.base_color = visual.own_colour;
  visual.paint.metallic = 0.35f;
  visual.paint.roughness = 0.3f;
  visual.glass_material = material_rough(vec3(0.12f, 0.16f, 0.22f), 0.1f);
  visual.glass_material.metallic = 0.6f;
  visual.light_material = material_rough(vec3(0.95f, 0.9f, 0.7f), 0.3f);
  visual.rear_light_material = material_rough(vec3(0.8f, 0.08f, 0.05f), 0.3f);
  visual.trim_material = material_rough(vec3(0.12f, 0.12f, 0.13f), 0.7f);
  visual.rubber = material_rough(vec3(0.08f, 0.08f, 0.08f), 0.9f);
  return visual;
}

void car_visual_destroy(car_visual_t* visual, renderer_t* renderer) {
  mesh_handle_t optional[] = {
      visual->chassis, visual->cabin,       visual->glass,
      visual->lights,  visual->rear_lights, visual->trim,
      visual->accent,  visual->beacon_a,    visual->beacon_b};
  for (size_t i = 0; i < sizeof(optional) / sizeof(optional[0]); i++)
    if (optional[i].id != 0) renderer_release_mesh(renderer, optional[i]);
  renderer_release_mesh(renderer, visual->wheel);
  renderer_release_mesh(renderer, visual->smoke_puff);
  renderer_release_mesh(renderer, visual->dashboard);
  renderer_release_mesh(renderer, visual->wheel_rim);
  renderer_release_mesh(renderer, visual->wheel_hub);
}

/* The shared body parts: paint, glass, lamps, trim, and for lorries and
 * emergency vehicles the accent colour and flashing beacons. */
static void draw_body(const car_visual_t* visual, renderer_t* renderer,
                      mat4_t model, material_t paint, material_t headlights,
                      material_t tail_lights, float clock) {
  draw_optional(renderer, visual->chassis, paint, model);
  draw_optional(renderer, visual->cabin, paint, model);
  draw_optional(renderer, visual->glass, visual->glass_material, model);
  draw_optional(renderer, visual->lights, headlights, model);
  draw_optional(renderer, visual->rear_lights, tail_lights, model);
  draw_optional(renderer, visual->trim, visual->trim_material, model);
  draw_optional(renderer, visual->accent, visual->accent_material, model);
  if (visual->emergency) {
    /* A negative clock: not on a call, the beacons stay dark. */
    bool first = fmodf(clock * 3.0f, 1.0f) < 0.5f;
    material_t a = visual->beacon_a_material, b = visual->beacon_b_material;
    a.emissive = clock >= 0 && first ? 3.0f : 0.1f;
    b.emissive = clock >= 0 && !first ? 3.0f : 0.1f;
    draw_optional(renderer, visual->beacon_a, a, model);
    draw_optional(renderer, visual->beacon_b, b, model);
  }
}

void car_visual_draw(const car_visual_t* visual, renderer_t* renderer,
                     const vehicle_t* vehicle, transform_t chassis,
                     float lights_on, float clock) {
  mat4_t model = transform_to_matrix(chassis);
  material_t headlights = visual->light_material;
  headlights.emissive = 0.15f + 1.6f * lights_on;
  material_t tail_lights = visual->rear_light_material;
  tail_lights.emissive = 0.1f + 0.9f * lights_on + 1.2f * vehicle->brake;
  material_t paint = visual->paint;
  float tint = car_visual_damage_tint(vehicle->damage);
  float grey = 0.3f * paint.base_color.x + 0.59f * paint.base_color.y +
               0.11f * paint.base_color.z;
  paint.base_color =
      vec3_add(vec3_scale(paint.base_color, tint),
               vec3_scale(vec3(grey, grey, grey), (1.0f - tint) * 0.6f));
  paint.roughness = fminf(paint.roughness + (1.0f - tint) * 0.5f, 1.0f);
  draw_body(visual, renderer, model, paint, headlights, tail_lights, clock);
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
    transform_t wheel = vehicle_wheel_transform(vehicle, chassis, i);
    renderer_draw_mesh(renderer, visual->wheel, visual->rubber,
                       transform_to_matrix(wheel));
    if (i >= 2) {
      /* Tandem axles turn with the driven one. */
      for (int k = i % 2; k < visual->extra_wheel_count; k += 2) {
        transform_t extra = wheel;
        extra.position =
            transform_point(chassis, visual->extra_wheel_centres[k]);
        renderer_draw_mesh(renderer, visual->wheel, visual->rubber,
                           transform_to_matrix(extra));
      }
    }
  }
}

void car_visual_draw_still(const car_visual_t* visual, renderer_t* renderer,
                           transform_t chassis, vec3_t paint, float lights_on,
                           float brake, float wheel_turn, float clock) {
  mat4_t model = transform_to_matrix(chassis);
  material_t body = visual->paint;
  body.base_color = visual->own_paint ? visual->own_colour : paint;
  material_t headlights = visual->light_material;
  headlights.emissive = 0.15f + 1.6f * lights_on;
  material_t tail_lights = visual->rear_light_material;
  tail_lights.emissive = 0.1f + 0.9f * lights_on + 1.2f * brake;
  draw_body(visual, renderer, model, body, headlights, tail_lights, clock);
  quat_t spin = quat_from_axis_angle(vec3(1, 0, 0), -wheel_turn);
  for (int i = 0; i < 4 + visual->extra_wheel_count; i++) {
    transform_t wheel = transform_identity();
    wheel.position =
        transform_point(chassis, i < 4 ? visual->wheel_centres[i]
                                       : visual->extra_wheel_centres[i - 4]);
    wheel.rotation = quat_multiply(chassis.rotation, spin);
    renderer_draw_mesh(renderer, visual->wheel, visual->rubber,
                       transform_to_matrix(wheel));
  }
}

void car_visual_draw_cockpit(const car_visual_t* visual, renderer_t* renderer,
                             transform_t chassis, float wheel_angle) {
  mat4_t car = transform_to_matrix(chassis);
  renderer_draw_mesh(renderer, visual->dashboard, visual->dash_material, car);
  /* The wheel stands before the driver's seat, its top tilted away. */
  vec3_t where = vec3(-visual->half_extents.x * 0.42f, 0.13f,
                      -visual->half_extents.z * 0.3f + 0.3f);
  quat_t tilt = quat_from_axis_angle(vec3(1, 0, 0), -0.55f);
  quat_t turn = quat_from_axis_angle(vec3(0, 0, 1), wheel_angle);
  mat4_t model = mat4_multiply(
      car,
      mat4_multiply(mat4_translation(where),
                    mat4_multiply(mat4_from_quat(tilt), mat4_from_quat(turn))));
  material_t rubber = material_rough(vec3(0.07f, 0.07f, 0.075f), 0.85f);
  renderer_draw_mesh(renderer, visual->wheel_rim, rubber, model);
  renderer_draw_mesh(renderer, visual->wheel_hub, visual->dash_material, model);
}

float car_visual_damage_tint(float damage) {
  if (damage <= 0) return 1.0f;
  return 1.0f - 0.6f * (damage > 1 ? 1 : damage);
}

void car_visual_draw_smoke(const car_visual_t* visual, renderer_t* renderer,
                           transform_t chassis, float damage, float time) {
  if (damage < 0.6f) return;
  int puffs = 6 + (int)((damage - 0.6f) * 10.0f);
  material_t grey = material_rough(vec3(0.2f, 0.2f, 0.21f), 1.0f);
  vec3_t nose =
      transform_point(chassis, vec3(0, 0.1f, -visual->half_extents.z * 0.75f));
  for (int i = 0; i < puffs; i++) {
    float phase = fmodf(time * 0.9f + (float)i / (float)puffs, 1.0f);
    float rise = phase * 2.4f;
    float drift = sinf((float)i * 2.3f + time) * 0.25f * phase;
    float size = 0.2f + phase * 0.9f;
    vec3_t at = vec3(nose.x + drift, nose.y + rise, nose.z + drift * 0.6f);
    mat4_t model = mat4_multiply(mat4_translation(at),
                                 mat4_scaling(vec3(size, size, size)));
    grey.base_color =
        vec3_scale(vec3(0.24f, 0.24f, 0.25f), 1.0f - 0.6f * phase);
    renderer_draw_mesh(renderer, visual->smoke_puff, grey, model);
  }
}
