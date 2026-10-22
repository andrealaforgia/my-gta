#include "mission_hud.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

mission_markers_t mission_markers_create(renderer_t* renderer) {
  mission_markers_t markers = {{0}, {0}};
  mesh_data_t pillar = mesh_data_create();
  mesh_data_add_cylinder_y(&pillar, vec3(0, 0, 0), 0.45f, 70.0f, 10);
  markers.pillar = renderer_upload_mesh(renderer, &pillar);
  mesh_data_free(&pillar);
  mesh_data_t ring = mesh_data_create();
  mesh_data_add_cylinder_y(&ring, vec3(0, 0, 0), MISSION_ARRIVAL_RADIUS * 0.8f,
                           0.12f, 24);
  markers.ring = renderer_upload_mesh(renderer, &ring);
  mesh_data_free(&ring);
  return markers;
}

void mission_markers_destroy(mission_markers_t* markers, renderer_t* renderer) {
  renderer_release_mesh(renderer, markers->pillar);
  renderer_release_mesh(renderer, markers->ring);
}

void mission_marker_draw(const mission_t* mission,
                         const mission_markers_t* markers, renderer_t* renderer,
                         float ground, float clock) {
  if (!mission->active) return;
  float pulse = 0.75f + 0.25f * sinf(clock * 4.0f);
  material_t light = material_plain(vec3(1.0f, 0.78f, 0.12f));
  light.emissive = 1.4f * pulse;
  mat4_t at = mat4_translation(vec3(mission->x, ground, mission->z));
  renderer_draw_mesh(renderer, markers->pillar, light, at);
  renderer_draw_mesh(renderer, markers->ring, light, at);
}

/* Text with a dark shadow, readable against anything. */
static void shadowed(renderer_t* renderer, float x, float y, float scale,
                     vec3_t colour, const char* text) {
  renderer_draw_text(renderer, x + scale * 0.5f, y + scale * 0.5f, scale,
                     vec3(0.02f, 0.02f, 0.03f), text);
  renderer_draw_text(renderer, x, y, scale, colour, text);
}

static void time_text(char* out, size_t size, float seconds) {
  int whole = seconds > 0 ? (int)seconds : 0;
  snprintf(out, size, "%d:%02d", whole / 60, whole % 60);
}

void mission_hud_draw(const mission_t* mission, renderer_t* renderer,
                      float screen_width, float screen_height, float x, float z,
                      float fx, float fz, float aim_x, float aim_z) {
  float scale = screen_width / 640.0f;
  if (scale < 2) scale = 2;
  texture_handle_t none = {0};
  char line[64];
  vec3_t white = vec3(1, 1, 1), gold = vec3(1.0f, 0.82f, 0.2f);
  float right = screen_width - 8.0f * scale;
  float top =
      8.0f * scale + 62.0f * scale + 10.0f * scale; /* under the minimap */
  snprintf(line, sizeof(line), "CASH %d", mission->cash);
  float text_scale = scale * 1.1f;
  shadowed(renderer, right - (float)strlen(line) * 6 * text_scale, top,
           text_scale, gold, line);
  if (mission->active) {
    float d = mission_distance_to(mission, x, z);
    if (d >= 1000.0f)
      snprintf(line, sizeof(line), "DELIVER %.1f KM", d / 1000.0f);
    else
      snprintf(line, sizeof(line), "DELIVER %d M", (int)d);
    shadowed(renderer, right - (float)strlen(line) * 6 * text_scale,
             top + 10 * text_scale, text_scale, white, line);
    char clock[16];
    time_text(clock, sizeof(clock), mission->time_left);
    snprintf(line, sizeof(line), "TIME %s", clock);
    vec3_t colour = mission->time_left < 15.0f ? vec3(1.0f, 0.3f, 0.2f) : white;
    shadowed(renderer, right - (float)strlen(line) * 6 * text_scale,
             top + 20 * text_scale, text_scale, colour, line);
    /* The arrow: up is the way the car points. */
    float tx = aim_x - x, tz = aim_z - z;
    float length = hypotf(tx, tz);
    if (length > 1.0f) {
      float forward_dot = (tx * fx + tz * fz) / length;
      float side_dot =
          (tx * -fz + tz * fx) / length; /* positive: to the right */
      float cx = right - 17.0f * scale,
            cy = top + 30.0f * text_scale + 20.0f * scale;
      float arm = 13.0f * scale;
      float ex = cx + side_dot * arm, ey = cy - forward_dot * arm;
      float bx = cx - side_dot * arm * 0.6f, by = cy + forward_dot * arm * 0.6f;
      renderer_draw_screen_image(renderer, none, cx - 17 * scale,
                                 cy - 17 * scale, 34 * scale, 34 * scale,
                                 vec3(0.04f, 0.05f, 0.07f), 0.55f);
      renderer_draw_screen_line(renderer, bx, by, ex, ey, 2.4f * scale, gold);
      /* The head: two short strokes back from the tip. */
      float hx = ex - bx, hy = ey - by;
      float hl = hypotf(hx, hy);
      if (hl > 1e-3f) {
        hx /= hl, hy /= hl;
        float head = 6.0f * scale;
        renderer_draw_screen_line(
            renderer, ex, ey, ex - hx * head - hy * head * 0.7f,
            ey - hy * head + hx * head * 0.7f, 2.2f * scale, gold);
        renderer_draw_screen_line(
            renderer, ex, ey, ex - hx * head + hy * head * 0.7f,
            ey - hy * head - hx * head * 0.7f, 2.2f * scale, gold);
      }
    }
  }
  if (mission->result_time > 0) {
    if (mission->last_reward > 0)
      snprintf(line, sizeof(line), "DELIVERED  +$%d", mission->last_reward);
    else
      snprintf(line, sizeof(line), "TOO LATE");
    float big = scale * 2.2f;
    float w = (float)strlen(line) * 6 * big;
    float px = (screen_width - w) * 0.5f, py = screen_height * 0.22f;
    vec3_t colour = mission->last_reward > 0 ? vec3(0.5f, 1.0f, 0.5f)
                                             : vec3(1.0f, 0.4f, 0.3f);
    shadowed(renderer, px + big * 0.4f, py + big * 0.4f, big, vec3(0, 0, 0),
             line);
    shadowed(renderer, px, py, big, colour, line);
  }
}

/* A diamond of four strokes, flashing. */
static void diamond(renderer_t* renderer, float cx, float cy, float r,
                    float width, vec3_t colour) {
  renderer_draw_screen_line(renderer, cx, cy - r, cx + r, cy, width, colour);
  renderer_draw_screen_line(renderer, cx + r, cy, cx, cy + r, width, colour);
  renderer_draw_screen_line(renderer, cx, cy + r, cx - r, cy, width, colour);
  renderer_draw_screen_line(renderer, cx - r, cy, cx, cy - r, width, colour);
}

void mission_minimap_mark(const mission_t* mission, renderer_t* renderer,
                          const city_map_t* map, float left, float top,
                          float side, float u0, float v0, float u1, float v1,
                          float scale, float clock) {
  if (!mission->active || map->size == 0) return;
  float px =
      left + ((mission->x - map->min_x) / map->extent - u0) / (u1 - u0) * side;
  float py =
      top + ((mission->z - map->min_z) / map->extent - v0) / (v1 - v0) * side;
  float edge = 3.0f * scale;
  px = fminf(fmaxf(px, left + edge), left + side - edge);
  py = fminf(fmaxf(py, top + edge), top + side - edge);
  float pulse = 0.8f + 0.25f * sinf(clock * 6.0f);
  diamond(renderer, px, py, 3.2f * scale * pulse, 1.8f * scale,
          vec3(0.1f, 0.1f, 0.1f));
  diamond(renderer, px, py, 2.6f * scale * pulse, 1.2f * scale,
          vec3(1.0f, 0.85f, 0.1f));
}

void mission_map_mark(const mission_t* mission, renderer_t* renderer,
                      const city_map_t* map, float left, float top, float side,
                      float scale, float clock) {
  if (!mission->active || map->size == 0) return;
  float k = side / map->extent;
  float px = left + (mission->x - map->min_x) * k;
  float py = top + (mission->z - map->min_z) * k;
  float pulse = 0.85f + 0.2f * sinf(clock * 6.0f);
  diamond(renderer, px, py, 8.0f * scale * pulse, 3.0f * scale,
          vec3(0.1f, 0.1f, 0.1f));
  diamond(renderer, px, py, 7.0f * scale * pulse, 1.8f * scale,
          vec3(1.0f, 0.85f, 0.1f));
}

static void route_segment(renderer_t* renderer, float x0, float y0, float x1,
                          float y1, float left, float top, float side,
                          float width) {
  /* Cut the segment to the window (Liang-Barsky). */
  float dx = x1 - x0, dy = y1 - y0;
  float p[4] = {-dx, dx, -dy, dy};
  float q[4] = {x0 - left, left + side - x0, y0 - top, top + side - y0};
  float t0 = 0, t1 = 1;
  for (int i = 0; i < 4; i++) {
    if (p[i] == 0) {
      if (q[i] < 0) return;
    } else {
      float t = q[i] / p[i];
      if (p[i] < 0) {
        if (t > t1) return;
        if (t > t0) t0 = t;
      } else {
        if (t < t0) return;
        if (t < t1) t1 = t;
      }
    }
  }
  renderer_draw_screen_line(renderer, x0 + dx * t0, y0 + dy * t0, x0 + dx * t1,
                            y0 + dy * t1, width, vec3(0.2f, 0.62f, 1.0f));
}

void mission_route_on_minimap(renderer_t* renderer, const city_map_t* map,
                              const float* line, size_t count, float left,
                              float top, float side, float u0, float v0,
                              float u1, float v1, float scale) {
  if (count < 2 || map->size == 0) return;
  float px = 0, py = 0;
  for (size_t i = 0; i < count; i++) {
    float x = left + ((line[2 * i] - map->min_x) / map->extent - u0) /
                         (u1 - u0) * side;
    float y = top + ((line[2 * i + 1] - map->min_z) / map->extent - v0) /
                        (v1 - v0) * side;
    if (i > 0)
      route_segment(renderer, px, py, x, y, left, top, side, 1.8f * scale);
    px = x, py = y;
  }
}

void mission_route_on_map(renderer_t* renderer, const city_map_t* map,
                          const float* line, size_t count, float left,
                          float top, float side, float scale) {
  if (count < 2 || map->size == 0) return;
  float k = side / map->extent;
  for (size_t i = 1; i < count; i++)
    route_segment(renderer, left + (line[2 * i - 2] - map->min_x) * k,
                  top + (line[2 * i - 1] - map->min_z) * k,
                  left + (line[2 * i] - map->min_x) * k,
                  top + (line[2 * i + 1] - map->min_z) * k, left, top, side,
                  2.2f * scale);
}
