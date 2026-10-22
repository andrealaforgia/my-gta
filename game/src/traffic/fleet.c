#include "fleet.h"

static const char* files[FLEET_VARIANTS] = {
    "assets/vehicles/hatchback.cfg", "assets/vehicles/sedan.cfg",
    "assets/vehicles/van.cfg",       "assets/vehicles/mini.cfg",
    "assets/vehicles/coupe.cfg",     "assets/vehicles/suv.cfg",
    "assets/vehicles/taxi.cfg",      "assets/vehicles/police.cfg",
    "assets/vehicles/ambulance.cfg", "assets/vehicles/fire_engine.cfg",
    "assets/vehicles/bus.cfg",       "assets/vehicles/box_truck.cfg",
    "assets/vehicles/tanker.cfg",    "assets/vehicles/flatbed.cfg",
    "assets/vehicles/refuse.cfg",    "assets/vehicles/pickup.cfg",
    "assets/vehicles/mixer.cfg"};

const char* fleet_vehicle_file(int variant) {
  return files[((variant % FLEET_VARIANTS) + FLEET_VARIANTS) % FLEET_VARIANTS];
}

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
  static const int table[20] = {0, 0, 0, 0, 1, 1, 1, 1, 3, 3,
                                3, 4, 4, 5, 5, 2, 2, 6, 0, 1};
  return table[((pick % 20) + 20) % 20];
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

int fleet_traffic_variant_for(int pick) {
  /* Per hundred: 70 cars, then police 3, ambulance 2, fire engine 1, bus 6,
   * box lorry 5, tanker 2, flatbed 3, refuse 2, pickup 4, mixer 2. */
  static const int tail[30] = {7,  7,  7,  8,  8,  9,  10, 10, 10, 10,
                               10, 10, 11, 11, 11, 11, 11, 12, 12, 13,
                               13, 13, 14, 14, 15, 15, 15, 15, 16, 16};
  int n = ((pick % 100) + 100) % 100;
  if (n < 70) return fleet_variant_for(n * 7 + pick / 100);
  return tail[n - 70];
}
