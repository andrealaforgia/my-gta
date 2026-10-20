/**
 * @file city_scene.h
 * @brief Loads a cooked city package (see docs/asset-pipeline.md) into
 * the world and physics world.
 */
#ifndef GAME_SRC_SCENES_CITY_SCENE_H_
#define GAME_SRC_SCENES_CITY_SCENE_H_

#include "scene.h"

bool scene_build_city(scene_t* scene, scene_context_t context,
                      const char* package_directory);

#endif  // GAME_SRC_SCENES_CITY_SCENE_H_
