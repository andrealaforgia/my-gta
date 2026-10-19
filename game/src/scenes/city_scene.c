#include "city_scene.h"

#include "log.h"

bool scene_build_city(scene_t* scene, scene_context_t context, const char* package_directory) {
  (void)scene;
  (void)context;
  LOG_ERROR("scene", "city packages are not loadable yet: %s", package_directory ? package_directory : "(none)");
  return false;
}
