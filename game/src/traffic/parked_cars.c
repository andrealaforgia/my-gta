#include "parked_cars.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

bool parked_cars_create(parked_cars_t* parked, const city_parking_t* spots,
                        size_t spot_count, physics_world_t* physics,
                        const fleet_t* fleet) {
  memset(parked, 0, sizeof(*parked));
  if (spot_count == 0) return true;
  parked->cars = calloc(spot_count, sizeof(parked_car_t));
  if (!parked->cars) return false;
  for (size_t i = 0; i < spot_count; i++) {
    ray_t down = {vec3(spots[i].x, 300.0f, spots[i].z), vec3(0, -1, 0)};
    ray_cast_result_t hit;
    if (!physics_ray_cast(physics, down, 600.0f, COLLISION_LAYER_GROUND, &hit))
      continue; /* nothing to stand on */
    parked_car_t* car = &parked->cars[parked->count];
    car->variant = fleet_variant_for(spots[i].variant);
    car->paint = spots[i].variant;
    car->yaw = spots[i].yaw;
    const car_visual_t* visual = &fleet->visuals[car->variant];
    car->position =
        vec3(spots[i].x, hit.hit.point.y + visual->ride_height, spots[i].z);
    car->collider = physics_add_static_box(
        physics, transform_at(car->position, quat_from_yaw(car->yaw)),
        visual->half_extents, COLLISION_LAYER_PROP, 0.5f, 0);
    parked->count++;
  }
  return true;
}

void parked_cars_destroy(parked_cars_t* parked, physics_world_t* physics) {
  for (size_t i = 0; i < parked->count; i++)
    physics_remove_collider(physics, parked->cars[i].collider);
  free(parked->cars);
  memset(parked, 0, sizeof(*parked));
}

int parked_cars_draw(const parked_cars_t* parked, const fleet_t* fleet,
                     renderer_t* renderer, vec3_t eye, float range,
                     float lights_on) {
  int drawn = 0;
  float range_squared = range * range;
  for (size_t i = 0; i < parked->count; i++) {
    const parked_car_t* car = &parked->cars[i];
    float dx = car->position.x - eye.x, dz = car->position.z - eye.z;
    if (dx * dx + dz * dz > range_squared) continue;
    car_visual_draw_still(&fleet->visuals[car->variant], renderer,
                          transform_at(car->position, quat_from_yaw(car->yaw)),
                          fleet_paint(car->paint), lights_on * 0.0f, 0.0f,
                          0.0f);
    drawn++;
  }
  return drawn;
}
