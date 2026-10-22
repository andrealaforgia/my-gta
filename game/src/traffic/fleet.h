/**
 * @file fleet.h
 * @brief The vehicles that are not the player's: seven cars (one a taxi), then
 * police, ambulance, fire engine, bus and six kinds of lorry, each built from
 * its own tuning file, and a palette of paints. Parked cars use only the first
 * seven; moving traffic uses them all.
 */
#ifndef GAME_SRC_TRAFFIC_FLEET_H_
#define GAME_SRC_TRAFFIC_FLEET_H_

#include <stdbool.h>

#include "car_visual.h"

#define FLEET_CAR_VARIANTS \
  7 /* hatchback, saloon, van, city car, coupe, SUV, taxi */
#define FLEET_VARIANTS 17
#define FLEET_PAINTS 10
#define FLEET_POLICE_VARIANT 7
#define FLEET_AMBULANCE_VARIANT 8
#define FLEET_BUS_VARIANT 10

typedef struct {
  car_visual_t visuals[FLEET_VARIANTS];
} fleet_t;

bool fleet_create(fleet_t* fleet, renderer_t* renderer);
void fleet_destroy(fleet_t* fleet, renderer_t* renderer);
/* Any number picks a car style and paint: styles weighted to mostly
 * hatchbacks and saloons. */
int fleet_variant_for(int pick);
/* Any number picks anything on the road: about seven in ten are cars, the
 * rest buses, lorries and the emergency services. */
int fleet_traffic_variant_for(int pick);
/* The tuning file of a variant. */
const char* fleet_vehicle_file(int variant);
vec3_t fleet_paint(int pick);

#endif  // GAME_SRC_TRAFFIC_FLEET_H_
