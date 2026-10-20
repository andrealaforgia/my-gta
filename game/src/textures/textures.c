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
  /* Courtyards and verges in a town: worn grey paving with soil patches. */
  return grainy(seed, 0.52f, 0.50f, 0.46f, 0.35f, 0.2f);
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
      {0.92f, 0.90f, 0.86f}, /* whitewash */
      {0.66f, 0.70f, 0.74f}, /* grey-blue */
      {0.76f, 0.40f, 0.32f}, /* brick red */
      {0.88f, 0.78f, 0.58f}, /* sand */
      {0.62f, 0.60f, 0.58f}, /* concrete */
      {0.82f, 0.70f, 0.50f}, /* straw yellow */
  };
  static const float shutter_colours[][3] = {
      {0.22f, 0.30f, 0.24f}, /* green */
      {0.36f, 0.24f, 0.16f}, /* brown */
      {0.34f, 0.38f, 0.44f}, /* slate blue */
      {0.56f, 0.20f, 0.18f}, /* oxblood */
  };
  const float* base = palette[seed % 12];
  const float* shutter_colour = shutter_colours[(seed / 3) % 4];
  uint8_t* pixels = grainy(seed, base[0], base[1], base[2], 0.16f, 0.06f);
  if (!pixels) return NULL;
  int cell_w = TEXTURE_SIZE / (window_columns > 0 ? window_columns : 1);
  int cell_h = TEXTURE_SIZE / (window_rows > 0 ? window_rows : 1);
  bool shutters_open = (seed % 3) != 0;
  bool arched = ((seed / 2) % 3) == 0;
  bool balcony = ((seed / 5) % 3) == 0;
  bool wide = ((seed / 7) % 4) == 0;
  bool dark_frames = ((seed / 11) % 3) == 0;
  float window_lo = wide ? 0.28f : 0.36f, window_hi = wide ? 0.72f : 0.64f;
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
      float top = 0.78f;
      bool in_window =
          fx > window_lo && fx < window_hi && fy > 0.22f && fy < top;
      if (arched && fy >= 0.62f && fy < 0.86f) {
        /* A round head on the window: an ellipse over its top quarter. */
        float centre = 0.5f, half = (window_hi - window_lo) * 0.5f;
        float t = (fy - 0.62f) / 0.24f;
        in_window = fx > centre - half * sqrtf(fmaxf(1.0f - t * t, 0.0f)) &&
                    fx < centre + half * sqrtf(fmaxf(1.0f - t * t, 0.0f));
      }
      bool sill = fx > window_lo - 0.04f && fx < window_hi + 0.04f &&
                  fy > 0.18f && fy <= 0.22f;
      bool rail = balcony && fy > 0.22f && fy < 0.34f &&
                  fx > window_lo - 0.06f && fx < window_hi + 0.06f &&
                  (fy < 0.25f || ((int)(fx * 40) % 3) == 0);
      bool shutter = shutters_open &&
                     ((fx > window_lo - 0.12f && fx < window_lo - 0.01f) ||
                      (fx > window_hi + 0.01f && fx < window_hi + 0.12f)) &&
                     fy > 0.22f && fy < top;
      if (rail) {
        put(pixels, x, y, 0.16f, 0.16f, 0.17f);
      } else if (sill) {
        put(pixels, x, y, 0.82f, 0.80f, 0.74f);
      } else if (shutter) {
        float slat = ((cy / 3) % 2) ? 0.9f : 1.0f;
        put(pixels, x, y, shutter_colour[0] * slat, shutter_colour[1] * slat,
            shutter_colour[2] * slat);
      } else if (in_window) {
        float inset = fminf(fminf(fx - window_lo, window_hi - fx), fy - 0.22f);
        bool frame = inset < 0.03f || (fx > 0.49f && fx < 0.51f) ||
                     (fy > 0.49f && fy < 0.51f);
        if (frame) {
          if (dark_frames)
            put(pixels, x, y, 0.28f, 0.20f, 0.14f);
          else
            put(pixels, x, y, 0.80f, 0.78f, 0.72f);
        } else {
          float sky = 0.3f + noise((unsigned)(x / cell_w),
                                   (unsigned)(y / cell_h), seed + 5) *
                                 0.15f;
          put(pixels, x, y, sky * 0.55f, sky * 0.65f, sky * 0.9f);
          /* Alpha marks glass: at night some panes are lit from inside. */
          pixels[(y * TEXTURE_SIZE + x) * 4 + 3] = 0;
        }
      }
    }
  }
  return pixels;
}

/* A painted house front in the manner of the Pordenone Corso: a ground of
 * ochre or rose washed with a lozenge pattern in a darker and a lighter
 * tone, framed windows with sills on top of it, one bay per repeat. */
uint8_t* texture_facade_fresco(unsigned seed) {
  uint8_t* pixels = texture_facade(seed, 1, 1);
  if (!pixels) return NULL;
  static const float tints[][3] = {
      {0.78f, 0.42f, 0.30f}, {0.86f, 0.66f, 0.40f}, {0.70f, 0.52f, 0.40f}};
  const float* tint = tints[seed % 3];
  for (int y = 0; y < TEXTURE_SIZE; y++)
    for (int x = 0; x < TEXTURE_SIZE; x++) {
      uint8_t* q = pixels + (y * TEXTURE_SIZE + x) * 4;
      /* Leave the window openings, shutters and string course alone: only
       * repaint the plaster, recognised by its warm, light colour. */
      bool plaster = q[0] > 150 && q[0] >= q[2] + 15 && q[1] > 110;
      if (!plaster) continue;
      int dx = (x % 32) - 16, dy = (y % 32) - 16;
      int manhattan = abs(dx) + abs(dy);
      float band = manhattan < 4
                       ? 1.12f
                       : (manhattan >= 15 && manhattan <= 17 ? 0.7f : 1.0f);
      float r = tint[0] * band, g = tint[1] * band, b = tint[2] * band;
      put(pixels, x, y, r, g, b);
    }
  return pixels;
}

/* Red brick in stretcher bond for the Municipio, windows kept. */
uint8_t* texture_facade_brick(unsigned seed) {
  uint8_t* pixels = texture_facade(seed, 1, 1);
  if (!pixels) return NULL;
  for (int y = 0; y < TEXTURE_SIZE; y++)
    for (int x = 0; x < TEXTURE_SIZE; x++) {
      uint8_t* q = pixels + (y * TEXTURE_SIZE + x) * 4;
      bool plaster = q[0] > 150 && q[0] >= q[2] + 15 && q[1] > 110;
      if (!plaster) continue;
      int row = y / 6, shifted = x + (row % 2) * 8;
      bool mortar = (y % 6) < 1 || (shifted % 16) < 1;
      float n =
          0.88f + 0.24f * noise((unsigned)(shifted / 16), (unsigned)row, seed);
      if (mortar)
        put(pixels, x, y, 0.72f, 0.68f, 0.62f);
      else
        put(pixels, x, y, 0.62f * n, 0.30f * n, 0.22f * n);
    }
  return pixels;
}

uint8_t* texture_roof_slate(unsigned seed) {
  uint8_t* pixels = grainy(seed, 0.34f, 0.35f, 0.38f, 0.2f, 0.1f);
  if (!pixels) return NULL;
  const int tile_w = 12, row_h = 12;
  for (int y = 0; y < TEXTURE_SIZE; y++)
    for (int x = 0; x < TEXTURE_SIZE; x++) {
      int row = y / row_h;
      int shifted = x + (row % 2) * tile_w / 2;
      float across = (float)(shifted % tile_w) / tile_w;
      float shade = across < 0.08f ? 0.7f : 1.0f;
      float edge = (y % row_h) < 2 ? 0.65f : 1.0f;
      uint8_t* q = pixels + (y * TEXTURE_SIZE + x) * 4;
      put(pixels, x, y, q[0] / 255.0f * shade * edge,
          q[1] / 255.0f * shade * edge, q[2] / 255.0f * shade * edge);
    }
  return pixels;
}

uint8_t* texture_roof_flat(unsigned seed) {
  uint8_t* pixels = grainy(seed, 0.42f, 0.42f, 0.41f, 0.3f, 0.18f);
  if (!pixels) return NULL;
  /* Bitumen sheets with seams every 64 px. */
  for (int y = 0; y < TEXTURE_SIZE; y++)
    for (int x = 0; x < TEXTURE_SIZE; x++)
      if (x % 64 < 2) {
        uint8_t* q = pixels + (y * TEXTURE_SIZE + x) * 4;
        put(pixels, x, y, q[0] / 255.0f * 0.7f, q[1] / 255.0f * 0.7f,
            q[2] / 255.0f * 0.7f);
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
  static const unsigned facade_seeds[CITY_FACADE_STYLES] = {
      36, 49, 62, 75, 88, 101, 114, 127, 140, 153, 166, 179};
  for (int i = 0; i < CITY_FACADE_STYLES; i++)
    set.facades[i] = upload(
        renderer, texture_facade(facade_seeds[i], 1, (i % 3) == 2 ? 2 : 1));
  set.facade = set.facades[0];
  set.roof = upload(renderer, texture_roof(6));
  set.facade_fresco = upload(renderer, texture_facade_fresco(5));
  set.facade_brick = upload(renderer, texture_facade_brick(23));
  set.roof_slate = upload(renderer, texture_roof_slate(8));
  set.roof_flat = upload(renderer, texture_roof_flat(9));
  set.checker = upload(renderer, texture_checker());
  return set;
}
