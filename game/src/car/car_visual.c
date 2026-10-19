#include "car_visual.h"

car_visual_t car_visual_create(renderer_t* renderer, const vehicle_params_t* p) {
  car_visual_t visual;
  vec3_t h = p->chassis_half_extents;
  mesh_data_t chassis = mesh_data_create();
  /* Lower body, then a narrower cabin set back from the nose (-Z). */
  mesh_data_add_box(&chassis, vec3(0, 0, 0), vec3(h.x, h.y * 0.8f, h.z));
  mesh_data_add_box(&chassis, vec3(0, h.y * 0.8f + h.y * 0.75f, h.z * 0.12f),
                    vec3(h.x * 0.85f, h.y * 0.75f, h.z * 0.45f));
  /* Bonnet wedge as a thin box so the nose reads from the chase camera. */
  mesh_data_add_box(&chassis, vec3(0, h.y * 0.8f + 0.03f, -h.z * 0.7f), vec3(h.x * 0.9f, 0.03f, h.z * 0.25f));
  visual.chassis = renderer_upload_mesh(renderer, &chassis);
  mesh_data_free(&chassis);

  mesh_data_t wheel = mesh_data_create();
  mesh_data_add_cylinder_x(&wheel, vec3_zero(), p->wheel_radius, p->wheel_width, 20);
  /* A hub cap so the spin is visible. */
  mesh_data_add_box(&wheel, vec3_zero(), vec3(p->wheel_width * 0.55f, p->wheel_radius * 0.15f, p->wheel_radius * 0.6f));
  visual.wheel = renderer_upload_mesh(renderer, &wheel);
  mesh_data_free(&wheel);

  visual.paint = material_plain(vec3(0.75f, 0.12f, 0.1f));
  visual.paint.metallic = 0.3f;
  visual.paint.roughness = 0.35f;
  visual.rubber = material_rough(vec3(0.08f, 0.08f, 0.08f), 0.9f);
  return visual;
}

void car_visual_destroy(car_visual_t* visual, renderer_t* renderer) {
  renderer_release_mesh(renderer, visual->chassis);
  renderer_release_mesh(renderer, visual->wheel);
}

void car_visual_draw(const car_visual_t* visual, renderer_t* renderer, const vehicle_t* vehicle,
                     transform_t chassis) {
  renderer_draw_mesh(renderer, visual->chassis, visual->paint, transform_to_matrix(chassis));
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
    transform_t wheel = vehicle_wheel_transform(vehicle, chassis, i);
    renderer_draw_mesh(renderer, visual->wheel, visual->rubber, transform_to_matrix(wheel));
  }
}
