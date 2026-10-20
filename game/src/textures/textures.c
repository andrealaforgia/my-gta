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
uint8_t* texture_earth(unsigned seed) {
  return grainy(seed, 0.50f, 0.45f, 0.37f, 0.35f, 0.2f);
}
uint8_t* texture_roof(unsigned seed) {
  uint8_t* pixels = grainy(seed, 0.72f, 0.42f, 0.30f, 0.25f, 0.12f);
  if (!pixels) return NULL;
  /* Rows of coppi: each tile a half-cylinder shaded across its width,
   * rows overlapping with a shadow line. */
  const int tile_w = 16, row_h = 24;
  for (int y = 0; y < TEXTURE_SIZE; y++) {
    for (int x = 0; x < TEXTURE_SIZE; x++) {
      int row = y / row_h;
      int shifted = x + (row % 2) * tile_w / 2;
      float across = (float)(shifted % tile_w) / tile_w;
      float shade = 0.75f + 0.35f * sinf(across * 3.14159f);
      float edge = (y % row_h) < 3 ? 0.6f : 1.0f;
      uint8_t* p = pixels + (y * TEXTURE_SIZE + x) * 4;
      float r = p[0] / 255.0f * shade * edge, g = p[1] / 255.0f * shade * edge,
            b = p[2] / 255.0f * shade * edge;
      put(pixels, x, y, r, g, b);
    }
  }
  return pixels;
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

/* A rendered Friulian facade: one storey per tile vertically and one
 * window bay per tile horizontally. Plaster in one of a few town colours,
 * a window with a stone sill and dark shutters, a string course near the
 * top of the tile. Seeds pick the colour and the shutter style. */
uint8_t* texture_facade(unsigned seed, int window_rows, int window_columns) {
  static const float palette[][3] = {
      {0.86f, 0.74f, 0.52f}, /* ochre */
      {0.80f, 0.52f, 0.40f}, /* terracotta */
      {0.90f, 0.86f, 0.76f}, /* cream */
      {0.84f, 0.66f, 0.60f}, /* faded pink */
      {0.78f, 0.80f, 0.70f}, /* pale sage */
      {0.70f, 0.62f, 0.52f}, /* old stone */
  };
  const float* base = palette[seed % 6];
  uint8_t* pixels = grainy(seed, base[0], base[1], base[2], 0.16f, 0.06f);
  if (!pixels) return NULL;
  int cell_w = TEXTURE_SIZE / (window_columns > 0 ? window_columns : 1);
  int cell_h = TEXTURE_SIZE / (window_rows > 0 ? window_rows : 1);
  bool shutters_open = (seed % 2) == 0;
  for (int y = 0; y < TEXTURE_SIZE; y++) {
    for (int x = 0; x < TEXTURE_SIZE; x++) {
      int cx = x % cell_w, cy = y % cell_h;
      float fx = (float)cx / cell_w, fy = (float)cy / cell_h;
      /* v runs up the wall: fy near 1 is the top of the storey. */
      bool string_course = fy > 0.93f && fy < 0.97f;
      if (string_course) {
        put(pixels, x, y, base[0] * 0.78f, base[1] * 0.78f, base[2] * 0.76f);
        continue;
      }
      bool in_window = fx > 0.36f && fx < 0.64f && fy > 0.22f && fy < 0.78f;
      bool sill = fx > 0.32f && fx < 0.68f && fy > 0.18f && fy <= 0.22f;
      bool shutter = shutters_open ? ((fx > 0.24f && fx < 0.35f) ||
                                      (fx > 0.65f && fx < 0.76f)) &&
                                         fy > 0.22f && fy < 0.78f
                                   : false;
      if (sill) {
        put(pixels, x, y, 0.82f, 0.80f, 0.74f);
      } else if (shutter) {
        float slat = ((cy / 3) % 2) ? 0.9f : 1.0f;
        put(pixels, x, y, 0.22f * slat, 0.30f * slat, 0.24f * slat);
      } else if (in_window) {
        bool frame = fx < 0.39f || fx > 0.61f || fy < 0.25f || fy > 0.75f ||
                     (fx > 0.49f && fx < 0.51f) || (fy > 0.49f && fy < 0.51f);
        if (frame) {
          put(pixels, x, y, 0.80f, 0.78f, 0.72f);
        } else {
          float sky = 0.3f + noise((unsigned)(x / cell_w),
                                   (unsigned)(y / cell_h), seed + 5) *
                                 0.15f;
          put(pixels, x, y, sky * 0.55f, sky * 0.65f, sky * 0.9f);
        }
      }
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
  set.earth = upload(renderer, texture_earth(17));
  set.facade = upload(renderer, texture_facade(5, 1, 1));
  set.facades[0] = set.facade;
  set.facades[1] = upload(renderer, texture_facade(7, 1, 1));
  set.facades[2] = upload(renderer, texture_facade(11, 1, 2));
  set.facades[3] = upload(renderer, texture_facade(13, 1, 1));
  set.roof = upload(renderer, texture_roof(6));
  set.checker = upload(renderer, texture_checker());
  return set;
}
