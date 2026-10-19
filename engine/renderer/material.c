#include "renderer.h"

material_t material_plain(vec3_t base_color) {
  material_t m = {base_color, 0.0f, 0.8f, {0}};
  return m;
}

material_t material_rough(vec3_t base_color, float roughness) {
  material_t m = {base_color, 0.0f, roughness, {0}};
  return m;
}
