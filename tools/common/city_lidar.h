/**
 * @file city_lidar.h
 * @brief Lifts a city model from airborne laser heights: the terrain grid
 * from a terrain model, and the heights of buildings nobody mapped a height
 * for from the surface model minus the terrain model.
 */
#ifndef TOOLS_COMMON_CITY_LIDAR_H_
#define TOOLS_COMMON_CITY_LIDAR_H_

#include <stdbool.h>
#include <stddef.h>

#include "city_model.h"
#include "lidar.h"

/* The window of a model, in UTM 33N metres, with a margin: what to read from
 * the grids so a large file is not loaded whole. */
lidar_window_t city_model_utm_window(const city_model_t* model, double margin);

/* Replaces the terrain with the model sampled every `spacing` metres over
 * the city and a margin, relative to its lowest point. False if the terrain
 * set does not cover the city. */
bool city_model_apply_dtm(city_model_t* model, const lidar_set_t* dtm,
                          double spacing);

/* Sets the wall height of every building whose height is a guess (no height
 * or levels tag) to the eaves height read from the surface model minus the
 * terrain model inside its footprint. Returns how many changed. */
size_t city_model_apply_dsm(city_model_t* model, const lidar_set_t* dsm,
                            const lidar_set_t* dtm);

#endif  // TOOLS_COMMON_CITY_LIDAR_H_
