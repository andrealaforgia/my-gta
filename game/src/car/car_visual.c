#include "car_visual.h"

#include <stddef.h>

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
  visual.chassis = upload(renderer, &body_mesh);
  visual.cabin = upload(renderer, &cabin_mesh);
  visual.glass = upload(renderer, &glass_mesh);
  visual.lights = upload(renderer, &lights_mesh);
  visual.rear_lights = upload(renderer, &rear_mesh);
  visual.trim = upload(renderer, &trim_mesh);
  visual.wheel = upload(renderer, &wheel);
  visual.paint = material_plain(vec3(0.72f, 0.1f, 0.08f));
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
  renderer_release_mesh(renderer, visual->chassis);
  renderer_release_mesh(renderer, visual->cabin);
  renderer_release_mesh(renderer, visual->glass);
  renderer_release_mesh(renderer, visual->lights);
  renderer_release_mesh(renderer, visual->rear_lights);
  renderer_release_mesh(renderer, visual->trim);
  renderer_release_mesh(renderer, visual->wheel);
}

void car_visual_draw(const car_visual_t* visual, renderer_t* renderer,
                     const vehicle_t* vehicle, transform_t chassis,
                     float lights_on) {
  mat4_t model = transform_to_matrix(chassis);
  material_t headlights = visual->light_material;
  headlights.emissive = 0.15f + 1.6f * lights_on;
  material_t tail_lights = visual->rear_light_material;
  tail_lights.emissive = 0.1f + 0.9f * lights_on + 1.2f * vehicle->brake;
  renderer_draw_mesh(renderer, visual->chassis, visual->paint, model);
  renderer_draw_mesh(renderer, visual->cabin, visual->paint, model);
  renderer_draw_mesh(renderer, visual->glass, visual->glass_material, model);
  renderer_draw_mesh(renderer, visual->lights, headlights, model);
  renderer_draw_mesh(renderer, visual->rear_lights, tail_lights, model);
  renderer_draw_mesh(renderer, visual->trim, visual->trim_material, model);
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
    transform_t wheel = vehicle_wheel_transform(vehicle, chassis, i);
    renderer_draw_mesh(renderer, visual->wheel, visual->rubber,
                       transform_to_matrix(wheel));
  }
}

void car_visual_draw_still(const car_visual_t* visual, renderer_t* renderer,
                           transform_t chassis, vec3_t paint, float lights_on,
                           float brake, float wheel_turn) {
  mat4_t model = transform_to_matrix(chassis);
  material_t body = visual->paint;
  body.base_color = paint;
  material_t headlights = visual->light_material;
  headlights.emissive = 0.15f + 1.6f * lights_on;
  material_t tail_lights = visual->rear_light_material;
  tail_lights.emissive = 0.1f + 0.9f * lights_on + 1.2f * brake;
  renderer_draw_mesh(renderer, visual->chassis, body, model);
  renderer_draw_mesh(renderer, visual->cabin, body, model);
  renderer_draw_mesh(renderer, visual->glass, visual->glass_material, model);
  renderer_draw_mesh(renderer, visual->lights, headlights, model);
  renderer_draw_mesh(renderer, visual->rear_lights, tail_lights, model);
  renderer_draw_mesh(renderer, visual->trim, visual->trim_material, model);
  quat_t spin = quat_from_axis_angle(vec3(1, 0, 0), -wheel_turn);
  for (int i = 0; i < 4; i++) {
    transform_t wheel = transform_identity();
    wheel.position = transform_point(chassis, visual->wheel_centres[i]);
    wheel.rotation = quat_multiply(chassis.rotation, spin);
    renderer_draw_mesh(renderer, visual->wheel, visual->rubber,
                       transform_to_matrix(wheel));
  }
}
