/**
 * @file vehicle_bodies.h
 * @brief The body shapes that are not a plain car: police cars, ambulances,
 * fire engines, buses and the lorries (box, tanker, flatbed, refuse,
 * pickup, mixer). Each is a few boxes, prisms and glass quads built in the
 * vehicle frame (nose towards -Z, y up) from the chassis half extents.
 */
#ifndef GAME_SRC_CAR_VEHICLE_BODIES_H_
#define GAME_SRC_CAR_VEHICLE_BODIES_H_

#include <stdbool.h>

#include "mesh_data.h"
#include "vec3.h"

typedef enum {
  BODY_CAR,
  BODY_POLICE,
  BODY_AMBULANCE,
  BODY_FIRE_ENGINE,
  BODY_BUS,
  BODY_BOX_TRUCK,
  BODY_TANKER,
  BODY_FLATBED,
  BODY_REFUSE,
  BODY_PICKUP,
  BODY_MIXER,
  BODY_STYLE_COUNT
} body_style_t;

body_style_t body_style_from_name(const char* name);
const char* body_style_name(body_style_t style);

/* The meshes of a body: paint takes the vehicle's colour, accent a fixed
 * one, beacons flash. Meshes a style does not use stay empty. */
typedef struct {
  mesh_data_t body, cabin, glass, lights, rear, trim, accent;
  mesh_data_t beacon_a, beacon_b; /* the two flashing lights */
  vec3_t beacon_a_colour, beacon_b_colour;
  bool own_paint;       /* the paint is fixed, not picked from the palette */
  vec3_t paint;         /* when own_paint */
  vec3_t accent_colour; /* the accent mesh */
  int extra_axles;      /* tandem rear axles behind the driven one */
  float extra_axle_z;   /* metres behind the rear axle for the first */
} body_build_t;

/* Builds the whole body of a non-car style (everything but BODY_CAR and
 * BODY_POLICE) from the chassis half extents. */
bool vehicle_body_build(body_style_t style, vec3_t half, body_build_t* out);

/* Adds police livery and a light bar to a car body whose cabin roof is at
 * `roof` and spans z in [roof_front, roof_back]. */
bool vehicle_body_add_police(body_build_t* out, vec3_t half, float floor,
                             float roof, float roof_front, float roof_back);

void body_build_free(body_build_t* out);
void body_build_init(body_build_t* out);

#endif  // GAME_SRC_CAR_VEHICLE_BODIES_H_
