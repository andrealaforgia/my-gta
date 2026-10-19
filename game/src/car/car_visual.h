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
  mesh_handle_t wheel;
  material_t paint;
  material_t rubber;
} car_visual_t;

car_visual_t car_visual_create(renderer_t* renderer, const vehicle_params_t* params);
void car_visual_destroy(car_visual_t* visual, renderer_t* renderer);
void car_visual_draw(const car_visual_t* visual, renderer_t* renderer, const vehicle_t* vehicle,
                     transform_t chassis);

#endif  // GAME_SRC_CAR_CAR_VISUAL_H_
