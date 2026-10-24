#include "guide_hud.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* Text with a dark shadow, readable against anything. */
static void shadowed(renderer_t* renderer, float x, float y, float scale,
                     vec3_t colour, const char* text) {
  renderer_draw_text(renderer, x + scale * 0.5f, y + scale * 0.5f, scale,
                     vec3(0.02f, 0.02f, 0.03f), text);
  renderer_draw_text(renderer, x, y, scale, colour, text);
}

void guide_hud_draw(const guide_t* guide, renderer_t* renderer,
                    float screen_width, float screen_height, float x, float z,
                    float fx, float fz, float aim_x, float aim_z) {
  (void)screen_height;
  if (!guide->active) return;
  float scale = screen_width / 640.0f;
  if (scale < 2) scale = 2;
  texture_handle_t none = {0};
  char line[64];
  vec3_t white = vec3(1, 1, 1), gold = vec3(1.0f, 0.82f, 0.2f);
  float right = screen_width - 8.0f * scale;
  float top =
      8.0f * scale + 62.0f * scale + 10.0f * scale; /* under the minimap */
  float text_scale = scale * 1.1f;
  float d = hypotf(guide->x - x, guide->z - z);
  if (d >= 1000.0f)
    snprintf(line, sizeof(line), "AIRPORT %.1f KM", d / 1000.0f);
  else
    snprintf(line, sizeof(line), "AIRPORT %d M", (int)d);
  shadowed(renderer, right - (float)strlen(line) * 6 * text_scale, top,
           text_scale, white, line);
  /* The arrow: up is the way the player faces. */
  float tx = aim_x - x, tz = aim_z - z;
  float length = hypotf(tx, tz);
  if (length > 1.0f) {
    float forward_dot = (tx * fx + tz * fz) / length;
    float side_dot = (tx * -fz + tz * fx) / length; /* positive: to the right */
    float cx = right - 17.0f * scale,
          cy = top + 10.0f * text_scale + 20.0f * scale;
    float arm = 13.0f * scale;
    float ex = cx + side_dot * arm, ey = cy - forward_dot * arm;
    float bx = cx - side_dot * arm * 0.6f, by = cy + forward_dot * arm * 0.6f;
    renderer_draw_screen_image(renderer, none, cx - 17 * scale, cy - 17 * scale,
                               34 * scale, 34 * scale,
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

/* A diamond of four strokes, flashing. */
static void diamond(renderer_t* renderer, float cx, float cy, float r,
                    float width, vec3_t colour) {
  renderer_draw_screen_line(renderer, cx, cy - r, cx + r, cy, width, colour);
  renderer_draw_screen_line(renderer, cx + r, cy, cx, cy + r, width, colour);
  renderer_draw_screen_line(renderer, cx, cy + r, cx - r, cy, width, colour);
  renderer_draw_screen_line(renderer, cx - r, cy, cx, cy - r, width, colour);
}

void guide_minimap_mark(const guide_t* guide, renderer_t* renderer,
                        const city_map_t* map, float left, float top,
                        float side, float u0, float v0, float u1, float v1,
                        float scale, float clock) {
  if (!guide->active || map->size == 0) return;
  float px =
      left + ((guide->x - map->min_x) / map->extent - u0) / (u1 - u0) * side;
  float py =
      top + ((guide->z - map->min_z) / map->extent - v0) / (v1 - v0) * side;
  float edge = 3.0f * scale;
  px = fminf(fmaxf(px, left + edge), left + side - edge);
  py = fminf(fmaxf(py, top + edge), top + side - edge);
  float pulse = 0.8f + 0.25f * sinf(clock * 6.0f);
  diamond(renderer, px, py, 3.2f * scale * pulse, 1.8f * scale,
          vec3(0.1f, 0.1f, 0.1f));
  diamond(renderer, px, py, 2.6f * scale * pulse, 1.2f * scale,
          vec3(1.0f, 0.85f, 0.1f));
}

void guide_map_mark(const guide_t* guide, renderer_t* renderer,
                    const city_map_t* map, float left, float top, float side,
                    float scale, float clock) {
  if (!guide->active || map->size == 0) return;
  float k = side / map->extent;
  float px = left + (guide->x - map->min_x) * k;
  float py = top + (guide->z - map->min_z) * k;
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

void guide_route_on_minimap(renderer_t* renderer, const city_map_t* map,
                            const float* line, size_t count, float left,
                            float top, float side, float u0, float v0, float u1,
                            float v1, float scale) {
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

void guide_route_on_map(renderer_t* renderer, const city_map_t* map,
                        const float* line, size_t count, float left, float top,
                        float side, float scale) {
  if (count < 2 || map->size == 0) return;
  float k = side / map->extent;
  for (size_t i = 1; i < count; i++)
    route_segment(renderer, left + (line[2 * i - 2] - map->min_x) * k,
                  top + (line[2 * i - 1] - map->min_z) * k,
                  left + (line[2 * i] - map->min_x) * k,
                  top + (line[2 * i + 1] - map->min_z) * k, left, top, side,
                  2.2f * scale);
}

/* A police car's dot: red and blue taking turns, in a dark ring. */
static void police_dot(renderer_t* renderer, float x, float y, float radius,
                       float clock, size_t index) {
  bool red = fmodf(clock * 4.0f + (float)index * 0.37f, 1.0f) < 0.5f;
  texture_handle_t none = {0};
  renderer_draw_screen_image(renderer, none, x - radius * 1.3f,
                             y - radius * 1.3f, radius * 2.6f, radius * 2.6f,
                             vec3(0.02f, 0.02f, 0.04f), 1.0f);
  renderer_draw_screen_image(
      renderer, none, x - radius, y - radius, radius * 2.0f, radius * 2.0f,
      red ? vec3(1.0f, 0.15f, 0.1f) : vec3(0.2f, 0.4f, 1.0f), 1.0f);
}

void police_minimap_marks(renderer_t* renderer, const city_map_t* map,
                          const float* cars, size_t count, float left,
                          float top, float side, float u0, float v0, float u1,
                          float v1, float scale, float clock) {
  if (map->size == 0) return;
  float edge = 2.5f * scale;
  for (size_t i = 0; i < count; i++) {
    float px = left + ((cars[2 * i] - map->min_x) / map->extent - u0) /
                          (u1 - u0) * side;
    float py = top + ((cars[2 * i + 1] - map->min_z) / map->extent - v0) /
                         (v1 - v0) * side;
    px = fminf(fmaxf(px, left + edge), left + side - edge);
    py = fminf(fmaxf(py, top + edge), top + side - edge);
    police_dot(renderer, px, py, 1.7f * scale, clock, i);
  }
}

void police_map_marks(renderer_t* renderer, const city_map_t* map,
                      const float* cars, size_t count, float left, float top,
                      float side, float scale, float clock) {
  if (map->size == 0) return;
  float k = side / map->extent;
  for (size_t i = 0; i < count; i++)
    police_dot(renderer, left + (cars[2 * i] - map->min_x) * k,
               top + (cars[2 * i + 1] - map->min_z) * k, 3.0f * scale, clock,
               i);
}
