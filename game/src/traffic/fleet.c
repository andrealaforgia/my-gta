#include "fleet.h"

static const char* files[FLEET_VARIANTS] = {
    "assets/vehicles/hatchback.cfg", "assets/vehicles/sedan.cfg",
    "assets/vehicles/van.cfg", "assets/vehicles/mini.cfg"};

bool fleet_create(fleet_t* fleet, renderer_t* renderer) {
  for (int i = 0; i < FLEET_VARIANTS; i++) {
    vehicle_params_t params;
    if (!vehicle_params_load(&params, files[i])) return false;
    fleet->visuals[i] = car_visual_create(renderer, &params);
  }
  return true;
}

void fleet_destroy(fleet_t* fleet, renderer_t* renderer) {
  for (int i = 0; i < FLEET_VARIANTS; i++)
    car_visual_destroy(&fleet->visuals[i], renderer);
}

int fleet_variant_for(int pick) {
  /* Hatchbacks and saloons dominate, then city cars, and the odd van. */
  static const int table[10] = {0, 0, 0, 1, 1, 1, 3, 3, 0, 2};
  return table[((pick % 10) + 10) % 10];
}

vec3_t fleet_paint(int pick) {
  static const float paints[FLEET_PAINTS][3] = {
      {0.88f, 0.88f, 0.86f}, /* white */
      {0.62f, 0.64f, 0.66f}, /* silver */
      {0.06f, 0.06f, 0.07f}, /* black */
      {0.30f, 0.31f, 0.33f}, /* graphite */
      {0.10f, 0.16f, 0.36f}, /* dark blue */
      {0.62f, 0.08f, 0.08f}, /* red */
      {0.10f, 0.26f, 0.16f}, /* racing green */
      {0.72f, 0.66f, 0.52f}, /* beige */
      {0.38f, 0.07f, 0.13f}, /* bordeaux */
      {0.55f, 0.68f, 0.80f}, /* light blue */
  };
  const float* p =
      paints[((pick / 7) % FLEET_PAINTS + FLEET_PAINTS) % FLEET_PAINTS];
  return vec3(p[0], p[1], p[2]);
}
