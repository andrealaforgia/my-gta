/**
 * @file car_visual.h
 * @brief Procedural car meshes sized from the vehicle parameters, drawn
 * on the interpolated chassis and wheel transforms.
 */
#ifndef GAME_SRC_CAR_CAR_VISUAL_H_
#define GAME_SRC_CAR_CAR_VISUAL_H_

#include "renderer.h"
#include "vehicle.h"

typedef struct {
  mesh_handle_t chassis;
  mesh_handle_t cabin;
  mesh_handle_t glass;
  mesh_handle_t lights;
  mesh_handle_t rear_lights;
  mesh_handle_t trim;
  mesh_handle_t wheel;
  material_t paint;
  material_t glass_material;
  material_t light_material;
  material_t rear_light_material;
  material_t trim_material;
  material_t rubber;
  vec3_t wheel_centres[4]; /* hubs at rest, chassis frame: FL FR RL RR */
  float wheel_radius;
  float ride_height;   /* chassis centre above the road on its wheels */
  vec3_t half_extents; /* of the chassis box */
  mesh_handle_t dashboard, wheel_rim, wheel_hub; /* the inside */
  material_t dash_material;
} car_visual_t;

car_visual_t car_visual_create(renderer_t* renderer,
                               const vehicle_params_t* params);
void car_visual_destroy(car_visual_t* visual, renderer_t* renderer);
void car_visual_draw(const car_visual_t* visual, renderer_t* renderer,
                     const vehicle_t* vehicle, transform_t chassis,
                     float lights_on /* 0 by day .. 1 at night */);

/* Draws a car standing or rolling with no suspension state: the body in the
 * given paint, wheels at rest, lights as asked (brake lights light up
 * brightly). Used for parked and passing cars. */
void car_visual_draw_still(const car_visual_t* visual, renderer_t* renderer,
                           transform_t chassis, vec3_t paint, float lights_on,
                           float brake, float wheel_turn);

/* The driver's view of the inside: a dashboard across the base of the
 * windscreen and a steering wheel turned by `wheel_angle` radians. */
void car_visual_draw_cockpit(const car_visual_t* visual, renderer_t* renderer,
                             transform_t chassis, float wheel_angle);

#endif  // GAME_SRC_CAR_CAR_VISUAL_H_
