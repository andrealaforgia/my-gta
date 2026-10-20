#include "textures.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

static unsigned hash(unsigned x, unsigned y, unsigned seed) {
  unsigned h = x * 374761393u + y * 668265263u + seed * 2246822519u;
  h = (h ^ (h >> 13)) * 1274126177u;
  return h ^ (h >> 16);
}

static float noise(unsigned x, unsigned y, unsigned seed) {
  return (float)(hash(x, y, seed) & 0xffff) / 65535.0f;
}

/* Value noise with a few octaves, tiling at TEXTURE_SIZE. */
static float smooth_noise(int x, int y, unsigned seed, int cell) {
  int x0 = (x / cell) % (TEXTURE_SIZE / cell),
      y0 = (y / cell) % (TEXTURE_SIZE / cell);
  int x1 = (x0 + 1) % (TEXTURE_SIZE / cell),
      y1 = (y0 + 1) % (TEXTURE_SIZE / cell);
  float tx = (float)(x % cell) / (float)cell,
        ty = (float)(y % cell) / (float)cell;
  tx = tx * tx * (3 - 2 * tx);
  ty = ty * ty * (3 - 2 * ty);
  float a = noise((unsigned)x0, (unsigned)y0, seed),
        b = noise((unsigned)x1, (unsigned)y0, seed);
  float c = noise((unsigned)x0, (unsigned)y1, seed),
        d = noise((unsigned)x1, (unsigned)y1, seed);
  return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
}

static float fractal(int x, int y, unsigned seed) {
  return smooth_noise(x, y, seed, 64) * 0.5f +
         smooth_noise(x, y, seed + 1, 16) * 0.3f +
         smooth_noise(x, y, seed + 2, 4) * 0.2f;
}

static uint8_t* allocate(void) {
  return malloc(TEXTURE_SIZE * TEXTURE_SIZE * 4);
}

static void put(uint8_t* pixels, int x, int y, float r, float g, float b) {
  uint8_t* p = pixels + (y * TEXTURE_SIZE + x) * 4;
  p[0] = (uint8_t)(fminf(fmaxf(r, 0), 1) * 255);
  p[1] = (uint8_t)(fminf(fmaxf(g, 0), 1) * 255);
  p[2] = (uint8_t)(fminf(fmaxf(b, 0), 1) * 255);
  p[3] = 255;
}

static uint8_t* grainy(unsigned seed, float r, float g, float b,
                       float variation, float grain) {
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  for (int y = 0; y < TEXTURE_SIZE; y++) {
    for (int x = 0; x < TEXTURE_SIZE; x++) {
      float shade = 1.0f + (fractal(x, y, seed) - 0.5f) * variation +
                    (noise((unsigned)x, (unsigned)y, seed + 9) - 0.5f) * grain;
      put(pixels, x, y, r * shade, g * shade, b * shade);
    }
  }
  return pixels;
}

uint8_t* texture_asphalt(unsigned seed) {
  return grainy(seed, 0.30f, 0.30f, 0.31f, 0.3f, 0.25f);
}
uint8_t* texture_concrete(unsigned seed) {
  return grainy(seed, 0.62f, 0.60f, 0.56f, 0.25f, 0.12f);
}
uint8_t* texture_grass(unsigned seed) {
  return grainy(seed, 0.28f, 0.42f, 0.18f, 0.5f, 0.3f);
}
uint8_t* texture_roof(unsigned seed) {
  return grainy(seed, 0.36f, 0.30f, 0.28f, 0.3f, 0.15f);
}

uint8_t* texture_road(unsigned seed) {
  /* u runs across the road (0 left edge, 1 right edge), v along it. */
  uint8_t* pixels = texture_asphalt(seed);
  if (!pixels) return NULL;
  for (int y = 0; y < TEXTURE_SIZE; y++) {
    for (int x = 0; x < TEXTURE_SIZE; x++) {
      float u = (float)x / TEXTURE_SIZE;
      bool edge = (u > 0.03f && u < 0.045f) || (u > 0.955f && u < 0.97f);
      bool centre = fabsf(u - 0.5f) < 0.008f && (y % 64) < 32;
      if (edge || centre) {
        float wear = 0.75f + noise((unsigned)x, (unsigned)y, seed + 3) * 0.25f;
        put(pixels, x, y, 0.85f * wear, 0.85f * wear, 0.8f * wear);
      }
    }
  }
  return pixels;
}

uint8_t* texture_facade(unsigned seed, int window_rows, int window_columns) {
  float tint = noise(1, 2, seed);
  float r = 0.55f + tint * 0.3f, g = 0.45f + tint * 0.25f,
        b = 0.38f + tint * 0.2f;
  if ((seed % 3) == 1) {
    r = 0.78f;
    g = 0.76f;
    b = 0.70f;
  }
  uint8_t* pixels = grainy(seed, r, g, b, 0.2f, 0.08f);
  if (!pixels) return NULL;
  int cell_w = TEXTURE_SIZE / window_columns,
      cell_h = TEXTURE_SIZE / window_rows;
  for (int y = 0; y < TEXTURE_SIZE; y++) {
    for (int x = 0; x < TEXTURE_SIZE; x++) {
      int cx = x % cell_w, cy = y % cell_h;
      bool in_window = cx > cell_w * 0.25f && cx < cell_w * 0.75f &&
                       cy > cell_h * 0.2f && cy < cell_h * 0.75f;
      if (!in_window) continue;
      unsigned which =
          hash((unsigned)(x / cell_w), (unsigned)(y / cell_h), seed);
      float lit = (which % 7 == 0) ? 0.9f : 0.0f;
      float sky = 0.35f + noise((unsigned)(x / cell_w), (unsigned)(y / cell_h),
                                seed + 5) *
                              0.2f;
      bool frame = cx < cell_w * 0.28f || cx > cell_w * 0.72f ||
                   cy < cell_h * 0.23f || cy > cell_h * 0.72f;
      if (frame)
        put(pixels, x, y, 0.2f, 0.2f, 0.22f);
      else
        put(pixels, x, y, sky * 0.6f + lit * 0.9f, sky * 0.7f + lit * 0.8f,
            sky + lit * 0.5f);
    }
  }
  return pixels;
}

uint8_t* texture_checker(void) {
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  for (int y = 0; y < TEXTURE_SIZE; y++)
    for (int x = 0; x < TEXTURE_SIZE; x++) {
      bool dark = ((x / 32) + (y / 32)) % 2 == 0;
      put(pixels, x, y, dark ? 0.35f : 0.75f, dark ? 0.35f : 0.75f,
          dark ? 0.35f : 0.75f);
    }
  return pixels;
}

static texture_handle_t upload(renderer_t* renderer, uint8_t* pixels) {
  texture_handle_t none = {0};
  if (!pixels) return none;
  texture_handle_t handle =
      renderer_upload_texture(renderer, TEXTURE_SIZE, TEXTURE_SIZE, pixels);
  free(pixels);
  return handle;
}

texture_set_t texture_set_upload(renderer_t* renderer) {
  texture_set_t set;
  set.asphalt = upload(renderer, texture_asphalt(1));
  set.road = upload(renderer, texture_road(2));
  set.concrete = upload(renderer, texture_concrete(3));
  set.grass = upload(renderer, texture_grass(4));
  set.facade = upload(renderer, texture_facade(5, 4, 4));
  set.facades[0] = set.facade;
  set.facades[1] = upload(renderer, texture_facade(7, 3, 4));
  set.facades[2] = upload(renderer, texture_facade(11, 5, 4));
  set.facades[3] = upload(renderer, texture_facade(13, 4, 3));
  set.roof = upload(renderer, texture_roof(6));
  set.checker = upload(renderer, texture_checker());
  return set;
}
