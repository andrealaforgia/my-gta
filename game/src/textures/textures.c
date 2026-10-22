#include "textures.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "font5x7.h"

/* Everything here is painted per pixel from noise: periodic gradient noise
 * for broad and fine variation, cellular noise for stones, tiles and grains,
 * and hand-placed features (joints, cracks, frames) with the shading a
 * photograph would show: edges that catch light on one side and shadow the
 * other, grime that runs down under sills, wear where feet and tyres go.
 * Values are authored as colours would look in a photograph (sRGB); the
 * renderer converts them to light when it samples them. All textures tile. */

#define S TEXTURE_SIZE

static unsigned hash(unsigned x, unsigned y, unsigned seed) {
  unsigned h = x * 374761393u + y * 668265263u + seed * 2246822519u;
  h = (h ^ (h >> 13)) * 1274126177u;
  return h ^ (h >> 16);
}

/* 0..1 per integer cell. */
static float noise(unsigned x, unsigned y, unsigned seed) {
  return (float)(hash(x, y, seed) & 0xffff) / 65535.0f;
}

static float clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
static float mix(float a, float b, float t) { return a + (b - a) * t; }
static float smoothstep(float a, float b, float x) {
  float t = clamp01((x - a) / (b - a));
  return t * t * (3.0f - 2.0f * t);
}

/* Periodic gradient noise: `px` by `py` cells across the texture, about
 * -0.5 to 0.5 scaled to 0..1 around 0.5. */
static float perlin_xy(float x, float y, int px, int py, unsigned seed) {
  x = fmodf(fmodf(x, (float)S) + (float)S, (float)S);
  y = fmodf(fmodf(y, (float)S) + (float)S, (float)S);
  float fx = x / (float)S * (float)px, fy = y / (float)S * (float)py;
  int ix = (int)floorf(fx), iy = (int)floorf(fy);
  float tx = fx - (float)ix, ty = fy - (float)iy;
  float u = tx * tx * tx * (tx * (tx * 6 - 15) + 10);
  float v = ty * ty * ty * (ty * (ty * 6 - 15) + 10);
  float corner[4];
  for (int k = 0; k < 4; k++) {
    int cx = (ix + (k & 1)) % px, cy = (iy + (k >> 1)) % py;
    unsigned h = hash((unsigned)cx, (unsigned)cy, seed);
    float angle = (float)(h & 0xffff) / 65535.0f * 6.2831853f;
    float dx = tx - (float)(k & 1), dy = ty - (float)(k >> 1);
    corner[k] = cosf(angle) * dx + sinf(angle) * dy;
  }
  float top = mix(corner[0], corner[1], u),
        bottom = mix(corner[2], corner[3], u);
  return clamp01(0.5f + mix(top, bottom, v) * 1.25f);
}

static float perlin(float x, float y, int period, unsigned seed) {
  return perlin_xy(x, y, period, period, seed);
}

/* Layered noise, 0..1, from `period` cells with each octave twice as fine. */
static float fbm(float x, float y, int period, int octaves, unsigned seed) {
  float sum = 0, amplitude = 0.5f, total = 0;
  for (int o = 0; o < octaves; o++) {
    sum += amplitude * perlin(x, y, period << o, seed + (unsigned)o * 7919u);
    total += amplitude;
    amplitude *= 0.5f;
  }
  return sum / total;
}

/* Cellular noise on a wrapped grid of `cells` by `cells`: the distance to the
 * nearest point (in cell widths) and an id for that cell in 0..1. */
static float worley(float x, float y, int cells, unsigned seed, float* id,
                    float* second) {
  float fx = x / (float)S * (float)cells, fy = y / (float)S * (float)cells;
  int ix = (int)floorf(fx), iy = (int)floorf(fy);
  float best = 9, next = 9, best_id = 0;
  for (int dy = -1; dy <= 1; dy++)
    for (int dx = -1; dx <= 1; dx++) {
      int cx = ix + dx, cy = iy + dy;
      unsigned wx = (unsigned)((cx % cells + cells) % cells);
      unsigned wy = (unsigned)((cy % cells + cells) % cells);
      float px = (float)cx + 0.15f + 0.7f * noise(wx, wy, seed);
      float py = (float)cy + 0.15f + 0.7f * noise(wx, wy, seed + 1);
      float d = hypotf(px - fx, py - fy);
      if (d < best) {
        next = best;
        best = d;
        best_id = noise(wx, wy, seed + 2);
      } else if (d < next) {
        next = d;
      }
    }
  if (id) *id = best_id;
  if (second) *second = next;
  return best;
}

static uint8_t* allocate(void) { return malloc((size_t)S * S * 4); }

static void put(uint8_t* pixels, int x, int y, float r, float g, float b) {
  uint8_t* p = pixels + ((size_t)y * S + x) * 4;
  p[0] = (uint8_t)(clamp01(r) * 255.0f + 0.5f);
  p[1] = (uint8_t)(clamp01(g) * 255.0f + 0.5f);
  p[2] = (uint8_t)(clamp01(b) * 255.0f + 0.5f);
  p[3] = 255;
}

static void get(const uint8_t* pixels, int x, int y, float* r, float* g,
                float* b) {
  const uint8_t* p =
      pixels + ((size_t)((y % S + S) % S) * S + (size_t)((x % S + S) % S)) * 4;
  *r = (float)p[0] / 255.0f, *g = (float)p[1] / 255.0f,
  *b = (float)p[2] / 255.0f;
}

/* Multiplies a pixel's colour; coordinates wrap, so features that run off an
 * edge continue on the other side. */
static void shade(uint8_t* pixels, int x, int y, float factor) {
  x = (x % S + S) % S;
  y = (y % S + S) % S;
  float r, g, b;
  get(pixels, x, y, &r, &g, &b);
  put(pixels, x, y, r * factor, g * factor, b * factor);
}

static void blend(uint8_t* pixels, int x, int y, float r, float g, float b,
                  float amount) {
  x = (x % S + S) % S;
  y = (y % S + S) % S;
  float pr, pg, pb;
  get(pixels, x, y, &pr, &pg, &pb);
  put(pixels, x, y, mix(pr, r, amount), mix(pg, g, amount), mix(pb, b, amount));
}

/* ---- asphalt, concrete, grass, earth ---- */

uint8_t* texture_asphalt(unsigned seed) {
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      float broad = fbm((float)x, (float)y, 3, 4, seed);
      float mid = fbm((float)x, (float)y, 14, 3, seed + 11);
      float id, second;
      float f1 = worley((float)x, (float)y, 128, seed + 21, &id, &second);
      /* Stones standing in the binder, each its own grey. */
      float stone = smoothstep(0.42f, 0.12f, f1) * (0.35f + 0.65f * id);
      float pit = smoothstep(0.1f, 0.0f, second - f1) * 0.5f;
      float fine = noise((unsigned)x, (unsigned)y, seed + 5);
      float tone = 0.235f + 0.07f * (broad - 0.5f) + 0.05f * (mid - 0.5f);
      tone += stone * 0.14f - pit * 0.05f;
      tone *= 0.9f + 0.2f * fine;
      float warm =
          (noise((unsigned)(x / 64), (unsigned)(y / 64), seed + 9) - 0.5f) *
          0.02f;
      put(pixels, x, y, tone + warm, tone, tone * 1.03f - warm);
    }
  return pixels;
}

/* A wandering crack, dark in the middle and shaded at the lips. */
static void crack(uint8_t* pixels, float x, float y, float angle, int length,
                  float width, unsigned seed, int depth) {
  for (int step = 0; step < length; step++) {
    angle += (noise((unsigned)step, (unsigned)depth, seed) - 0.5f) * 0.55f;
    x += cosf(angle) * 1.4f;
    y += sinf(angle) * 1.4f;
    for (int dy = -2; dy <= 2; dy++)
      for (int dx = -2; dx <= 2; dx++) {
        float d = hypotf((float)dx, (float)dy);
        if (d > width + 1.5f) continue;
        float amount = d < width ? 0.65f : 0.2f;
        shade(pixels, (int)x + dx, (int)y + dy, 1.0f - amount);
      }
    if (depth < 2 && step > 10 && step % 23 == 0 &&
        noise((unsigned)step, (unsigned)depth + 5, seed + 3) > 0.85f)
      crack(pixels, x, y, angle + 0.9f * (step % 2 ? 1 : -1), length / 3,
            width * 0.7f, seed + (unsigned)step, depth + 1);
  }
}

uint8_t* texture_road(unsigned seed) {
  /* u runs across the road (0 left edge, 1 right edge), v along it. Two
   * lanes: the tyres polish a track an eighth and three eighths across each;
   * the surface has patched stretches, cracks, oil, and worn paint. */
  uint8_t* pixels = texture_asphalt(seed);
  if (!pixels) return NULL;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      float u = (float)x / S;
      float factor = 1.0f;
      float lane = fmodf(u * 2.0f, 1.0f);
      float track = fminf(fabsf(lane - 0.25f), fabsf(lane - 0.75f));
      if (track < 0.05f)
        factor *= 0.78f + 0.22f * smoothstep(0.0f, 0.05f, track);
      /* Oil dripped down the middle of the lanes. */
      float oil =
          smoothstep(0.62f, 0.8f, fbm((float)x, (float)y, 9, 3, seed + 61));
      if (track < 0.07f) factor *= 1.0f - 0.25f * oil;
      /* Patched stretches: blocks of newer or older tar with a seam. */
      int py = y / 96, px = x / 192;
      unsigned patch = hash((unsigned)py, (unsigned)px, seed + 17);
      float seam = 1.0f;
      if (patch % 9 == 0) {
        factor *= 0.82f;
        if (y % 96 < 2 || x % 192 < 2) seam = 0.6f;
      } else if (patch % 9 == 1) {
        factor *= 1.14f;
        if (y % 96 < 2 || x % 192 < 2) seam = 0.6f;
      }
      shade(pixels, x, y, factor * seam);
    }
  for (int c = 0; c < 3; c++)
    crack(pixels, (float)(hash((unsigned)c, 3, seed + 29) % S),
          (float)(hash((unsigned)c, 5, seed + 31) % S),
          1.5708f + (noise((unsigned)c, 9, seed) - 0.5f) * 0.6f, 170, 0.6f,
          seed + 41 + (unsigned)c, 0);
  /* Paint on top, worn: chipped by tyres and weather. */
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      float u = (float)x / S;
      float lane = fmodf(u * 2.0f, 1.0f);
      float track = fminf(fabsf(lane - 0.25f), fabsf(lane - 0.75f));
      bool edge = (u > 0.03f && u < 0.048f) || (u > 0.952f && u < 0.97f);
      float centre_edge = fabsf(u - 0.5f);
      bool centre = centre_edge < 0.009f && (y % 128) < 64;
      /* The dash ends round off a little. */
      if (centre && ((y % 128) < 3 || (y % 128) > 60))
        centre = centre_edge < 0.006f;
      if (!edge && !centre) continue;
      float chip = fbm((float)x * 2.0f, (float)y, 20, 4, seed + 71);
      float wear = smoothstep(0.55f, 0.78f, chip) * 0.75f;
      if (track < 0.04f) wear = fminf(wear + 0.25f, 0.9f);
      float edge_fade = 1.0f - smoothstep(0.003f, 0.009f, 0.009f - centre_edge);
      (void)edge_fade;
      float paint = 1.0f - wear;
      float grain = 0.9f + 0.1f * noise((unsigned)x, (unsigned)y, seed + 3);
      blend(pixels, x, y, 0.8f * grain, 0.8f * grain, 0.76f * grain, paint);
    }
  return pixels;
}

uint8_t* texture_concrete(unsigned seed) {
  /* Paving slabs a metre square, four to a repeat, with joints, pores,
   * stains and wear. */
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  const int slab = S / 4;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      /* The slabs are offset by half a slab so that the joints fall away from
       * the texture's edge and it tiles. */
      int jx = (x + slab / 2) % S, jy = (y + slab / 2) % S;
      int sx = jx / slab, sy = jy / slab;
      float slab_tone =
          (noise((unsigned)sx, (unsigned)sy, seed + 3) - 0.5f) * 0.1f;
      float broad = fbm((float)x, (float)y, 4, 4, seed);
      float mid = fbm((float)x, (float)y, 24, 3, seed + 5);
      float fine = noise((unsigned)x, (unsigned)y, seed + 7);
      float id, second;
      float pore_d = worley((float)x, (float)y, 170, seed + 13, &id, &second);
      float pore =
          smoothstep(0.16f, 0.04f, pore_d) * (id > 0.55f ? 1.0f : 0.0f);
      float tone =
          0.56f + slab_tone + 0.09f * (broad - 0.5f) + 0.07f * (mid - 0.5f);
      tone *= 0.93f + 0.14f * fine;
      tone *= 1.0f - 0.35f * pore;
      /* Damp, dirty patches near the joints. */
      int ex = jx % slab, ey = jy % slab;
      float edge = fminf(fminf((float)ex, (float)(slab - ex)),
                         fminf((float)ey, (float)(slab - ey)));
      tone *= 1.0f - 0.18f * (1.0f - smoothstep(0.0f, 14.0f, edge)) *
                         (0.4f + 0.6f * mid);
      /* The joint: a dark, dirt-filled line with a bright lip on one side. */
      if (ex < 2 || ey < 2)
        tone *= 0.42f;
      else if (ex == 2 || ey == 2)
        tone *= 1.1f;
      put(pixels, x, y, tone * 1.01f, tone, tone * 0.96f);
    }
  /* A few dark blotches: gum, oil, wet leaves. */
  for (int b = 0; b < 14; b++) {
    float cx = (float)(hash((unsigned)b, 1, seed + 91) % S);
    float cy = (float)(hash((unsigned)b, 2, seed + 92) % S);
    float r = 2.0f + (float)(hash((unsigned)b, 3, seed + 93) % 5);
    for (int dy = -8; dy <= 8; dy++)
      for (int dx = -8; dx <= 8; dx++) {
        float d = hypotf((float)dx, (float)dy);
        if (d < r)
          shade(pixels, (int)cx + dx, (int)cy + dy, 0.72f + 0.28f * d / r);
      }
  }
  return pixels;
}

uint8_t* texture_grass(unsigned seed) {
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      /* Blades: noise stretched along one axis, at several fineness. */
      float blades = perlin_xy((float)x, (float)y, 170, 26, seed) * 0.5f +
                     perlin_xy((float)x, (float)y, 90, 14, seed + 3) * 0.3f +
                     perlin_xy((float)x, (float)y, 260, 40, seed + 5) * 0.2f;
      float tufts = fbm((float)x, (float)y, 12, 3, seed + 17);
      float dry =
          smoothstep(0.6f, 0.82f, fbm((float)x, (float)y, 3, 3, seed + 23)) *
          0.8f;
      float depth = clamp01(blades * 1.3f - 0.15f);
      float r = mix(0.06f, 0.30f, depth), g = mix(0.12f, 0.46f, depth),
            b = mix(0.025f, 0.11f, depth);
      /* Dried and yellowed in places. */
      r = mix(r, 0.40f * (0.5f + depth), dry * 0.7f);
      g = mix(g, 0.36f * (0.5f + depth), dry * 0.7f);
      b = mix(b, 0.12f * (0.5f + depth), dry * 0.7f);
      float lump = 0.82f + 0.36f * tufts;
      float speck = noise((unsigned)x, (unsigned)y, seed + 29);
      /* Clover and the odd daisy. */
      float id, second;
      float cd = worley((float)x, (float)y, 36, seed + 31, &id, &second);
      if (cd < 0.16f && id > 0.7f) {
        r *= 0.8f;
        g *= 1.12f;
      }
      if (cd < 0.05f && id > 0.97f) {
        r = 0.9f;
        g = 0.88f;
        b = 0.7f;
      }
      put(pixels, x, y, r * lump * (0.92f + 0.16f * speck),
          g * lump * (0.92f + 0.16f * speck), b * lump);
    }
  return pixels;
}

uint8_t* texture_earth(unsigned seed) {
  /* Courtyards and verges: compacted gravel and dust, pebbles of many greys
   * and buffs standing in finer soil, with a few weeds. */
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      float id, second;
      float d = worley((float)x, (float)y, 70, seed + 3, &id, &second);
      float dust = fbm((float)x, (float)y, 5, 4, seed);
      float fine = noise((unsigned)x, (unsigned)y, seed + 8);
      float pebble = smoothstep(0.5f, 0.3f, d);
      float lip = smoothstep(0.05f, 0.0f, second - d);
      /* Each pebble its own colour: grey limestone, buff, brown. */
      float hue = id;
      float pr = mix(mix(0.50f, 0.58f, hue), 0.46f, hue > 0.7f ? 1.0f : 0.0f);
      float pg = mix(mix(0.48f, 0.52f, hue), 0.38f, hue > 0.7f ? 1.0f : 0.0f);
      float pb = mix(mix(0.44f, 0.42f, hue), 0.30f, hue > 0.7f ? 1.0f : 0.0f);
      float light = 0.75f + 0.5f * (1.0f - d) * pebble + (id - 0.5f) * 0.25f;
      float sr = 0.40f + 0.1f * dust, sg = 0.36f + 0.09f * dust,
            sb = 0.30f + 0.07f * dust;
      float r = mix(sr, pr * light, pebble), g = mix(sg, pg * light, pebble),
            b = mix(sb, pb * light, pebble);
      float k = (1.0f - 0.45f * lip) * (0.92f + 0.16f * fine);
      /* Dust drifts over it. */
      float drift =
          smoothstep(0.55f, 0.8f, fbm((float)x, (float)y, 3, 3, seed + 41));
      r = mix(r, 0.62f, drift * 0.3f);
      g = mix(g, 0.57f, drift * 0.3f);
      b = mix(b, 0.48f, drift * 0.3f);
      put(pixels, x, y, r * k, g * k, b * k);
    }
  return pixels;
}

/* ---- roofs ---- */

uint8_t* texture_roof(unsigned seed) {
  /* Coppi: Roman half-round clay tiles, a channel tile laid concave and a
   * cover tile convex over each join, courses overlapping down the slope.
   * Every tile fires to its own orange, some are brown or chalky; moss grows
   * in the channels and soot and rain streak them. */
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  const int tile_w = 32, course_h = 64;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      int course = y / course_h;
      int shifted = x + (course % 2) * (tile_w / 2) +
                    (int)(hash((unsigned)course, 4, seed) % 5);
      int column = shifted / tile_w;
      float across = (float)(shifted % tile_w) / (float)tile_w;
      float down =
          (float)(y % course_h) / (float)course_h; /* 0 at the top lip */
      unsigned id = hash((unsigned)column, (unsigned)course, seed);
      float variety = (float)(id & 0xff) / 255.0f;
      bool cover = (column % 2) == 0;
      /* A half-round: lit across its curve; channels are the shadowed
       * hollows between the covers. */
      float curve = sinf(across * 3.14159f);
      float lit = cover ? 0.55f + 0.6f * curve : 0.35f + 0.3f * (1.0f - curve);
      /* Courses overlap: a shadow under each lip, brighter at the free edge. */
      float lip =
          down < 0.12f ? 0.45f + 0.55f * smoothstep(0.0f, 0.12f, down) : 1.0f;
      float exposed = 0.9f + 0.2f * smoothstep(0.7f, 1.0f, down);
      float r = mix(0.62f, 0.50f, variety), g = mix(0.30f, 0.27f, variety),
            b = mix(0.19f, 0.17f, variety);
      if ((id >> 9) % 11 == 0) {
        r = 0.55f;
        g = 0.38f;
        b = 0.28f;
      } /* a pale replacement */
      if ((id >> 9) % 13 == 1) {
        r *= 0.7f;
        g *= 0.72f;
        b *= 0.75f;
      } /* an old dark one */
      float grain = 0.92f + 0.16f * fbm((float)x, (float)y, 60, 3, seed + 3);
      float shade_k = lit * lip * exposed * grain;
      r *= shade_k, g *= shade_k, b *= shade_k;
      /* Moss and lichen where it stays damp: the channels, under the lips. */
      float damp = (cover ? 0.15f : 0.8f) * (1.0f - down) +
                   0.3f * smoothstep(0.7f, 0.2f, down);
      float moss =
          smoothstep(0.55f, 0.75f, fbm((float)x, (float)y, 11, 4, seed + 7)) *
          damp;
      r = mix(r, 0.28f, moss * 0.55f);
      g = mix(g, 0.34f, moss * 0.55f);
      b = mix(b, 0.16f, moss * 0.55f);
      /* Soot and rain streaks down the slope. */
      float streak = perlin_xy((float)x, (float)y, 90, 3, seed + 13);
      float k = 1.0f - 0.1f * smoothstep(0.6f, 0.9f, streak);
      put(pixels, x, y, r * k, g * k, b * k);
    }
  return pixels;
}

uint8_t* texture_roof_slate(unsigned seed) {
  /* Slates in staggered courses, each a different blue-grey, lighter at its
   * free edge, with a shadow line under the course above. */
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  const int slate_w = 36, course_h = 28;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      int course = y / course_h;
      int shifted =
          x + (int)(hash((unsigned)course, 2, seed) % (unsigned)slate_w);
      int column = shifted / slate_w;
      float across = (float)(shifted % slate_w) / (float)slate_w;
      float down = (float)(y % course_h) / (float)course_h;
      unsigned id = hash((unsigned)column, (unsigned)course, seed + 5);
      float variety = (float)(id & 0xff) / 255.0f;
      float tone = 0.20f + 0.12f * variety;
      tone *= 0.8f + 0.35f * smoothstep(0.0f, 1.0f, down);
      if (down < 0.1f) tone *= 0.45f + 4.5f * down;
      if (across < 0.05f || across > 0.97f) tone *= 0.35f;
      /* Cleaved layers and wear. */
      float layers = perlin_xy((float)x, (float)y, 20, 180, seed + 9);
      tone *= 0.88f + 0.24f * layers;
      float wear =
          smoothstep(0.6f, 0.85f, fbm((float)x, (float)y, 8, 3, seed + 17));
      tone = mix(tone, tone * 1.5f, wear * 0.35f);
      float lichen =
          smoothstep(0.72f, 0.9f, fbm((float)x, (float)y, 13, 3, seed + 23));
      float r = tone * 0.96f, g = tone * 0.99f, b = tone * 1.08f;
      r = mix(r, 0.32f, lichen * 0.4f);
      g = mix(g, 0.35f, lichen * 0.4f);
      b = mix(b, 0.2f, lichen * 0.4f);
      put(pixels, x, y, r, g, b);
    }
  return pixels;
}

uint8_t* texture_roof_flat(unsigned seed) {
  /* A built-up roof: bitumen sheets with overlap seams, gravel ballast in
   * patches, ponding stains and the odd tuft of moss. */
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      float id, second;
      float d = worley((float)x, (float)y, 110, seed + 3, &id, &second);
      float gravel_area =
          smoothstep(0.45f, 0.62f, fbm((float)x, (float)y, 4, 4, seed));
      float stone = smoothstep(0.5f, 0.25f, d) * (0.3f + 0.7f * id);
      float membrane = 0.2f + 0.06f * fbm((float)x, (float)y, 12, 3, seed + 5);
      float tone = mix(membrane, 0.22f + 0.28f * stone, gravel_area);
      tone *= 0.9f + 0.2f * noise((unsigned)x, (unsigned)y, seed + 9);
      int sheet = (x + 64) % 128;
      if (sheet < 2)
        tone *= 0.55f;
      else if (sheet < 5)
        tone *= 1.1f;
      float pond =
          smoothstep(0.66f, 0.85f, fbm((float)x, (float)y, 5, 3, seed + 29));
      tone *= 1.0f - 0.15f * pond;
      float moss =
          smoothstep(0.74f, 0.88f, fbm((float)x, (float)y, 9, 3, seed + 37));
      put(pixels, x, y, mix(tone, 0.2f, moss * 0.5f),
          mix(tone, 0.26f, moss * 0.5f), mix(tone * 1.02f, 0.12f, moss * 0.5f));
    }
  return pixels;
}

/* ---- facades ---- */

typedef enum { WALL_STUCCO, WALL_FRESCO, WALL_BRICK } wall_kind_t;

/* The wall itself at a point: its colour before windows are cut into it. */
static void wall_colour(wall_kind_t kind, const float* base, int x, int y,
                        unsigned seed, float* r, float* g, float* b) {
  float mottle = fbm((float)x, (float)y, 9, 4, seed + 3);
  float broad = fbm((float)x, (float)y, 3, 3, seed + 5);
  float grain = noise((unsigned)x, (unsigned)y, seed + 7);
  if (kind == WALL_BRICK) {
    const int bw = 40, bh = 14;
    int row = y / bh;
    int shifted = x + (row % 2) * (bw / 2);
    int brick = shifted / bw;
    float bx = (float)(shifted % bw), by = (float)(y % bh);
    unsigned id = hash((unsigned)brick, (unsigned)row, seed);
    float v = (float)(id & 0xff) / 255.0f;
    float edge = fminf(fminf(bx, (float)bw - bx), fminf(by, (float)bh - by));
    bool mortar = bx < 3 || by < 3;
    if (mortar) {
      float m = 0.5f + 0.1f * mottle;
      *r = m, *g = m * 0.97f, *b = m * 0.9f;
      return;
    }
    float bevel = 0.8f + 0.2f * smoothstep(3.0f, 7.0f, edge);
    float r0 = mix(0.40f, 0.32f, v), g0 = mix(0.19f, 0.16f, v),
          b0 = mix(0.12f, 0.1f, v);
    if ((id >> 8) % 9 == 0) {
      r0 = 0.5f;
      g0 = 0.26f;
      b0 = 0.16f;
    }
    float k = bevel * (0.85f + 0.3f * grain) * (0.9f + 0.2f * mottle);
    *r = r0 * k, *g = g0 * k, *b = b0 * k;
    return;
  }
  float r0 = base[0], g0 = base[1], b0 = base[2];
  if (kind == WALL_FRESCO) {
    /* The painted lozenge pattern of the Corso: a darker and a lighter tone
     * in soft diamonds, aged. */
    int dx = (x % 64) - 32, dy = (y % 64) - 32;
    int man = abs(dx) + abs(dy);
    float band = man < 8 ? 1.14f : (man >= 30 && man <= 34 ? 0.72f : 1.0f);
    float soften = 0.97f + 0.06f * mottle;
    r0 *= band * soften, g0 *= band * soften, b0 *= band * soften;
  }
  /* Render: trowel marks, grains of sand, patches of damp and repair. */
  float k = 0.93f + 0.14f * mottle + 0.1f * (broad - 0.5f);
  k *= 0.95f + 0.1f * grain;
  float trowel = perlin_xy((float)x, (float)y, 40, 34, seed + 11);
  k *= 0.96f + 0.08f * trowel;
  *r = r0 * k, *g = g0 * k, *b = b0 * k;
}

static void paint_window(uint8_t* pixels, int x, int y, float fx, float fy,
                         float lo, float hi, bool arched, bool dark_frames,
                         bool shutters_open, const float* shutter, bool balcony,
                         unsigned seed, int cell_w, int cell_h) {
  float top = 0.78f;
  float cx = 0.5f, half = (hi - lo) * 0.5f;
  bool in_window = fx > lo && fx < hi && fy > 0.22f && fy < top;
  if (arched && fy >= 0.62f && fy < 0.86f) {
    float t = (fy - 0.62f) / 0.24f;
    float w = half * sqrtf(fmaxf(1.0f - t * t, 0.0f));
    in_window = fx > cx - w && fx < cx + w;
  }
  /* Stone surround of an arched window, with a keystone. */
  if (arched) {
    float t = clamp01((fy - 0.62f) / 0.24f);
    float w = half * sqrtf(fmaxf(1.0f - t * t, 0.0f));
    float out_d = (fabsf(fx - cx) - w);
    if (!in_window && fy > 0.62f && fy < 0.9f && out_d < 0.045f &&
        (fy > 0.62f ? true : false)) {
      float keystone = fabsf(fx - cx) < 0.02f && fy > 0.84f ? 1.1f : 1.0f;
      blend(pixels, x, y, 0.78f * keystone, 0.74f * keystone, 0.66f * keystone,
            0.9f);
    }
  }
  bool sill = fx > lo - 0.05f && fx < hi + 0.05f && fy > 0.17f && fy <= 0.22f;
  bool rail = balcony && fy > 0.22f && fy < 0.34f && fx > lo - 0.06f &&
              fx < hi + 0.06f && (fy < 0.235f || ((int)(fx * 46) % 5) == 0);
  bool shutter_area = shutters_open &&
                      ((fx > lo - 0.12f && fx < lo - 0.01f) ||
                       (fx > hi + 0.01f && fx < hi + 0.12f)) &&
                      fy > 0.22f && fy < top;
  /* Rain runs down from the sill over the wall below it. */
  if (fx > lo - 0.03f && fx < hi + 0.03f && fy < 0.17f) {
    float run = perlin_xy((float)x, (float)y, 70, 2, seed + 43);
    shade(pixels, x, y,
          1.0f - 0.12f * smoothstep(0.55f, 0.85f, run) *
                     (0.3f + 0.7f * (fy / 0.17f)));
  }
  if (rail) {
    put(pixels, x, y, 0.1f, 0.1f, 0.11f);
  } else if (sill) {
    /* Stone sill: a lit top edge, a shadowed underside. */
    float t = (fy - 0.17f) / 0.05f;
    float k = 0.7f + 0.4f * t;
    put(pixels, x, y, 0.78f * k, 0.75f * k, 0.68f * k);
  } else if (shutter_area) {
    /* Louvred shutters: slats that catch the light on their upper edge. */
    float slat = fmodf((float)(y % cell_h), 9.0f) / 9.0f;
    float k = 0.7f + 0.5f * smoothstep(0.0f, 0.3f, slat) *
                         (1.0f - smoothstep(0.7f, 1.0f, slat));
    float worn = 0.85f + 0.3f * fbm((float)x, (float)y, 25, 3, seed + 47);
    bool stile = fmodf(fx * 100.0f, 10.0f) < 1.0f;
    float edge = stile ? 0.7f : 1.0f;
    put(pixels, x, y, shutter[0] * k * worn * edge,
        shutter[1] * k * worn * edge, shutter[2] * k * worn * edge);
  } else if (in_window) {
    float inset = fminf(fminf(fx - lo, hi - fx), fy - 0.22f);
    bool bar = (fx > 0.49f && fx < 0.51f) || (fy > 0.49f && fy < 0.51f);
    bool frame = inset < 0.032f || bar;
    if (frame) {
      /* A painted wooden frame with a lit edge on its upper left. */
      float lighter = (fx < 0.5f ? 1.0f : 0.85f) * (fy > 0.5f ? 1.1f : 0.92f);
      if (dark_frames)
        put(pixels, x, y, 0.2f * lighter, 0.14f * lighter, 0.1f * lighter);
      else
        put(pixels, x, y, 0.86f * lighter, 0.84f * lighter, 0.78f * lighter);
    } else {
      /* Glass: dark, with the room behind it; alpha 0 so the shader can
       * mirror the sky or light the room. */
      float room = 0.09f + 0.05f * noise((unsigned)(x / cell_w),
                                         (unsigned)(y / cell_h), seed + 5);
      float reveal = smoothstep(0.032f, 0.1f, inset);
      put(pixels, x, y, room * 0.7f * (0.5f + 0.5f * reveal),
          room * 0.85f * (0.5f + 0.5f * reveal), room * (0.5f + 0.5f * reveal));
      pixels[((size_t)y * S + x) * 4 + 3] = 0;
    }
  } else if (fx > lo - 0.03f && fx < hi + 0.03f && fy > 0.2f &&
             fy < top + 0.03f) {
    /* The reveal: the thickness of the wall round the opening is in shadow. */
    float d = fminf(fminf(fabsf(fx - lo), fabsf(fx - hi)), fabsf(fy - top));
    if (d < 0.03f) shade(pixels, x, y, 0.75f + 0.25f * d / 0.03f);
  }
}

static uint8_t* facade_with(wall_kind_t kind, unsigned seed, int window_rows,
                            int window_columns) {
  static const float palette[][3] = {
      {0.72f, 0.60f, 0.42f}, /* ochre */
      {0.70f, 0.38f, 0.27f}, /* terracotta */
      {0.82f, 0.77f, 0.65f}, /* cream */
      {0.76f, 0.54f, 0.48f}, /* faded pink */
      {0.64f, 0.68f, 0.55f}, /* pale sage */
      {0.58f, 0.50f, 0.40f}, /* old stone */
      {0.86f, 0.84f, 0.79f}, /* whitewash */
      {0.52f, 0.58f, 0.64f}, /* grey-blue */
      {0.55f, 0.32f, 0.26f}, /* brick red */
      {0.80f, 0.68f, 0.45f}, /* sand */
      {0.55f, 0.53f, 0.50f}, /* concrete */
      {0.70f, 0.60f, 0.42f}, /* straw yellow */
  };
  static const float fresco_tints[][3] = {
      {0.68f, 0.34f, 0.24f}, {0.78f, 0.56f, 0.32f}, {0.60f, 0.42f, 0.32f}};
  static const float shutter_colours[][3] = {{0.15f, 0.26f, 0.18f},
                                             {0.28f, 0.17f, 0.10f},
                                             {0.26f, 0.31f, 0.38f},
                                             {0.45f, 0.14f, 0.12f}};
  const float* base =
      kind == WALL_FRESCO ? fresco_tints[seed % 3] : palette[seed % 12];
  const float* shutter = shutter_colours[(seed / 3) % 4];
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  int cell_w = S / (window_columns > 0 ? window_columns : 1);
  int cell_h = S / (window_rows > 0 ? window_rows : 1);
  bool shutters_open = (seed % 3) != 0;
  bool arched = ((seed / 2) % 3) == 0;
  bool balcony = ((seed / 5) % 3) == 0;
  bool wide = ((seed / 7) % 4) == 0;
  bool dark_frames = ((seed / 11) % 3) == 0;
  float lo = wide ? 0.28f : 0.36f, hi = wide ? 0.72f : 0.64f;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      float r, g, b;
      wall_colour(kind, base, x, y, seed, &r, &g, &b);
      put(pixels, x, y, r, g, b);
      int cx = x % cell_w, cy = y % cell_h;
      float fx = (float)cx / (float)cell_w, fy = (float)cy / (float)cell_h;
      /* A string course near the top of the storey: moulding with a lit
       * face over two shadowed steps. */
      if (fy > 0.91f && fy < 0.985f) {
        float t = (fy - 0.91f) / 0.075f;
        float k = t < 0.25f ? 0.65f : (t < 0.7f ? 1.05f : 0.8f);
        put(pixels, x, y, base[0] * 0.95f * k, base[1] * 0.93f * k,
            base[2] * 0.88f * k);
        continue;
      }
      /* Dirt collects below the moulding. */
      if (fy < 0.1f)
        shade(pixels, x, y, 0.9f + 0.1f * smoothstep(0.0f, 0.1f, fy));
      paint_window(pixels, x, y, fx, fy, lo, hi, arched, dark_frames,
                   shutters_open, shutter, balcony, seed, cell_w, cell_h);
    }
  /* Weathering over everything: damp rising from the foot, old streaks, and
   * here and there plaster fallen off to show the brick. */
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      uint8_t* p = pixels + ((size_t)y * S + x) * 4;
      if (p[3] == 0) continue;
      float streak = perlin_xy((float)x, (float)y, 120, 2, seed + 53);
      shade(pixels, x, y, 1.0f - 0.05f * smoothstep(0.62f, 0.95f, streak));
      if (kind == WALL_BRICK) continue;
      float loss = fbm((float)x, (float)y, 7, 3, seed + 59);
      if (loss > 0.8f && (seed % 4) == 1) {
        float edge = smoothstep(0.8f, 0.83f, loss);
        float v = noise((unsigned)(x / 12), (unsigned)(y / 5), seed + 61);
        blend(pixels, x, y, 0.5f + 0.1f * v, 0.26f + 0.06f * v, 0.18f,
              edge * 0.85f);
      }
    }
  return pixels;
}

uint8_t* texture_facade(unsigned seed, int window_rows, int window_columns) {
  return facade_with(WALL_STUCCO, seed, window_rows, window_columns);
}

uint8_t* texture_facade_fresco(unsigned seed) {
  return facade_with(WALL_FRESCO, seed, 1, 1);
}

uint8_t* texture_facade_brick(unsigned seed) {
  return facade_with(WALL_BRICK, seed, 1, 1);
}

/* ---- shop front, paving, door ---- */

/* A shop front, 4 m by one storey: a stone plinth, two display windows in
 * metal frames and a glazed door between them, a signboard with lettering
 * over a striped, folded awning that shades the wall under it. */
uint8_t* texture_shopfront(unsigned seed) {
  static const float awnings[][3] = {{0.62f, 0.10f, 0.10f},
                                     {0.10f, 0.32f, 0.17f},
                                     {0.10f, 0.18f, 0.40f},
                                     {0.72f, 0.5f, 0.08f}};
  static const float walls[][3] = {{0.70f, 0.64f, 0.50f},
                                   {0.52f, 0.46f, 0.38f},
                                   {0.78f, 0.74f, 0.66f},
                                   {0.24f, 0.24f, 0.26f}};
  const float* awning = awnings[seed % 4];
  const float* wall = walls[(seed / 4) % 4];
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      float fx = (float)x / S;
      float fy = 1.0f - (float)y / S;
      float r, g, b;
      wall_colour(WALL_STUCCO, wall, x, y, seed, &r, &g, &b);
      put(pixels, x, y, r, g, b);
      if (fy < 0.07f) {
        /* A plinth of dressed stone with a coping line. */
        float block = fmodf((float)x, 96.0f) < 2.0f ? 0.6f : 1.0f;
        float k = block * (0.8f + 0.3f * noise((unsigned)(x / 96), 1, seed));
        put(pixels, x, y, 0.46f * k, 0.44f * k, 0.4f * k);
        continue;
      }
      if (fy > 0.84f) {
        /* The signboard: dark painted wood with raised lettering. */
        float board = 0.1f + 0.04f * perlin_xy((float)x, (float)y, 8, 60, seed);
        bool border = fy > 0.97f || fy < 0.855f || fx < 0.02f || fx > 0.98f;
        if (border) {
          put(pixels, x, y, 0.55f, 0.46f, 0.25f);
        } else {
          float letters = 0;
          if (fy > 0.89f && fy < 0.95f && fx > 0.12f && fx < 0.88f) {
            int glyph = (x - (int)(0.12f * S)) / 14;
            unsigned gid = hash((unsigned)glyph, 3, seed);
            bool gap = glyph % 6 == 5;
            int gy = (int)((fy - 0.89f) / 0.06f * 7.0f);
            int gx = ((x - (int)(0.12f * S)) % 14) / 2;
            if (!gap && ((gid >> (gx + gy * 3)) & 1u)) letters = 1;
            if (!gap && (gx == 0 || gx == 5) && gy < 7) letters = 1;
          }
          put(pixels, x, y, board + letters * 0.75f, board + letters * 0.68f,
              board + letters * 0.5f);
        }
      } else if (fy > 0.68f) {
        /* The awning: stripes, with the fabric sagging between ribs and a
         * scalloped front edge; the wall beneath is shaded. */
        float sag = sinf(fx * 3.14159f * 12.0f);
        bool stripe = ((x / 22) % 2) == 0;
        float edge = (fy - 0.68f) / 0.16f; /* 0 at the lip */
        float scallop = fabsf(sinf((float)x / 22.0f * 3.14159f)) * 0.03f;
        if (edge < scallop * 6.0f) {
          /* Past the scalloped edge: the wall in its shadow. */
          shade(pixels, x, y, 0.55f);
        } else {
          float k = (0.62f + 0.38f * edge) * (0.9f + 0.1f * sag);
          float fr = stripe ? awning[0] : 0.82f, fg = stripe ? awning[1] : 0.8f,
                fb = stripe ? awning[2] : 0.74f;
          float weave =
              0.95f + 0.05f * noise((unsigned)x, (unsigned)y, seed + 3);
          put(pixels, x, y, fr * k * weave, fg * k * weave, fb * k * weave);
        }
      } else if (fy < 0.68f && fy > 0.62f) {
        shade(pixels, x, y, 0.6f + 0.4f * (0.62f - fy) / 0.06f * -1.0f + 0.4f);
      } else {
        bool window =
            fy > 0.1f && fy < 0.62f &&
            ((fx > 0.07f && fx < 0.43f) || (fx > 0.57f && fx < 0.93f));
        bool door = fy < 0.62f && fy > 0.07f && fx > 0.445f && fx < 0.555f;
        if (window || door) {
          float lo = window ? (fx < 0.5f ? 0.07f : 0.57f) : 0.445f;
          float hi = window ? (fx < 0.5f ? 0.43f : 0.93f) : 0.555f;
          float inset =
              fminf(fminf(fx - lo, hi - fx), fminf(fy - 0.1f, 0.62f - fy));
          bool frame = inset < 0.016f || (door && fy < 0.2f);
          if (frame) {
            float lit = fx < 0.5f ? 1.0f : 0.85f;
            put(pixels, x, y, 0.2f * lit, 0.19f * lit, 0.17f * lit);
          } else {
            float room = 0.1f + 0.07f * noise((unsigned)(x / 24),
                                              (unsigned)(y / 24), seed);
            put(pixels, x, y, room * 0.7f, room * 0.85f, room);
            pixels[((size_t)y * S + x) * 4 + 3] = 0;
          }
        }
      }
    }
  return pixels;
}

/* Stone flags, laid in courses of different heights, each flag its own
 * grey-buff or pinkish stone with bevelled edges, dirt-filled joints, pitting
 * and polish where feet pass; moss in a few joints. */
uint8_t* texture_paving(unsigned seed) {
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  int row_top[16], row_h[16], rows = 0, at = 0;
  while (at < S && rows < 16) {
    int h = 56 + (int)(hash((unsigned)rows, 7, seed) % 4) * 12;
    if (at + h > S || S - (at + h) < 40) h = S - at;
    row_top[rows] = at, row_h[rows] = h;
    at += h;
    rows++;
  }
  for (int y = 0; y < S; y++) {
    int row = 0;
    while (row + 1 < rows && y >= row_top[row + 1]) row++;
    int ry = y - row_top[row];
    int stone_w = 70 + (int)(hash((unsigned)row, 1, seed) % 4) * 22;
    int offset = (int)(hash((unsigned)row, 2, seed) % (unsigned)S);
    for (int x = 0; x < S; x++) {
      int along = (x + offset) % S;
      int per_row = S / stone_w;
      int stone = along / stone_w;
      (void)per_row;
      int sx = along % stone_w;
      /* The last stone in a course absorbs the remainder. */
      float stone_hue = noise((unsigned)stone, (unsigned)row, seed + 41);
      float tint = noise((unsigned)stone, (unsigned)row, seed + 43);
      float base = 0.5f + 0.14f * stone_hue;
      float edge = fminf(fminf((float)sx, (float)(stone_w - sx)),
                         fminf((float)ry, (float)(row_h[row] - ry)));
      float bevel = 0.78f + 0.22f * smoothstep(1.0f, 7.0f, edge);
      /* Light from the upper left: the left and top bevels brighter. */
      if (edge < 6.0f && (sx < 6 || ry < 6)) bevel *= 1.1f;
      float pits =
          noise((unsigned)x, (unsigned)y, seed + 47) > 0.985f ? 0.7f : 1.0f;
      float mottle = fbm((float)x, (float)y, 14, 4, seed + 51);
      float polish =
          1.0f + 0.12f * smoothstep(0.6f, 0.85f,
                                    fbm((float)x, (float)y, 4, 3, seed + 55));
      float k = base * bevel * pits * (0.85f + 0.3f * mottle) * polish;
      float r = k * (1.0f + (tint - 0.5f) * 0.12f), g = k,
            b = k * (1.0f - (tint - 0.5f) * 0.15f);
      bool joint = edge < 2.5f;
      if (joint) {
        float dirt = 0.18f + 0.06f * noise((unsigned)x, (unsigned)y, seed + 57);
        float moss =
            smoothstep(0.7f, 0.85f, fbm((float)x, (float)y, 16, 3, seed + 59));
        r = mix(dirt, 0.2f, moss * 0.6f);
        g = mix(dirt * 0.95f, 0.27f, moss * 0.6f);
        b = mix(dirt * 0.85f, 0.12f, moss * 0.6f);
      }
      put(pixels, x, y, r, g, b);
    }
  }
  return pixels;
}

/* A street door: a stone frame with a lintel, a painted wood leaf with grain
 * and four raised panels lit from the upper left, iron hinges, a brass handle
 * with a bright glint and a worn kick-plate area, a stone step below. */
uint8_t* texture_door(unsigned seed) {
  static const float woods[][3] = {{0.20f, 0.11f, 0.07f},
                                   {0.12f, 0.20f, 0.13f},
                                   {0.30f, 0.09f, 0.07f},
                                   {0.09f, 0.12f, 0.20f}};
  const float* wood = woods[seed % 4];
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      float fx = (float)x / S, fy = (float)y / S;
      /* Stone around the door. */
      float stone_mottle = fbm((float)x, (float)y, 12, 4, seed + 3);
      float block = fmodf((float)y, 64.0f) < 2.0f ? 0.6f : 1.0f;
      float st = (0.55f + 0.18f * stone_mottle) * block;
      put(pixels, x, y, st * 1.02f, st, st * 0.94f);
      bool leaf = fx > 0.14f && fx < 0.86f && fy > 0.1f && fy < 0.97f;
      if (fy >= 0.97f) {
        put(pixels, x, y, 0.45f, 0.43f, 0.4f);
        continue;
      }
      if (!leaf) {
        /* The frame's own bevel: shadow where it meets the leaf. */
        if (fx > 0.1f && fx < 0.14f && fy > 0.08f) shade(pixels, x, y, 0.65f);
        if (fy > 0.07f && fy < 0.1f && fx > 0.1f && fx < 0.9f)
          shade(pixels, x, y, 0.6f);
        continue;
      }
      /* Painted wood: long grain, brush marks, and wear at the foot. */
      float grain = perlin_xy((float)x, (float)y, 90, 3, seed + 5);
      float rings = perlin_xy((float)x, (float)y, 24, 2, seed + 7);
      float k = 0.82f + 0.3f * grain + 0.08f * rings;
      float wear = smoothstep(0.82f, 1.0f, fy) * 0.5f +
                   0.3f * smoothstep(0.7f, 0.95f,
                                     fbm((float)x, (float)y, 8, 3, seed + 9));
      bool panel = ((fx > 0.21f && fx < 0.47f) || (fx > 0.53f && fx < 0.79f)) &&
                   ((fy > 0.17f && fy < 0.46f) || (fy > 0.53f && fy < 0.92f));
      if (panel) {
        /* A raised panel: lit on its upper and left edges, shadowed on the
         * lower and right, flat in the middle. */
        float left = fx < 0.47f ? fx - 0.21f : fx - 0.53f;
        float right = fx < 0.47f ? 0.47f - fx : 0.79f - fx;
        float topd = fy < 0.5f ? fy - 0.17f : fy - 0.53f;
        float botd = fy < 0.5f ? 0.46f - fy : 0.92f - fy;
        float edge = fminf(fminf(left, right), fminf(topd, botd));
        if (edge < 0.022f) {
          float light = (left < 0.022f || topd < 0.022f) ? 1.35f : 0.6f;
          k *= light;
        } else {
          k *= 1.06f;
        }
      } else {
        k *= 0.92f;
      }
      k *= 1.0f - 0.4f * wear;
      float r = wood[0] * k, g = wood[1] * k, b = wood[2] * k;
      r = mix(r, 0.35f, wear * 0.4f), g = mix(g, 0.3f, wear * 0.4f),
      b = mix(b, 0.25f, wear * 0.4f);
      /* Hinges. */
      for (int h = 0; h < 3; h++) {
        float hy = 0.2f + 0.3f * (float)h;
        if (fabsf(fy - hy) < 0.012f && fx > 0.14f && fx < 0.23f)
          r = g = b = 0.06f;
      }
      /* The handle: a brass disc with a glint. */
      float dx = fx - 0.72f, dy = fy - 0.52f;
      float d = sqrtf(dx * dx + dy * dy);
      if (d < 0.034f) {
        float glint = smoothstep(0.014f, 0.0f, hypotf(dx + 0.01f, dy + 0.01f));
        r = 0.7f + 0.3f * glint, g = 0.52f + 0.4f * glint,
        b = 0.16f + 0.5f * glint;
      } else if (d < 0.038f) {
        r = g = b = 0.05f;
      }
      put(pixels, x, y, r, g, b);
    }
  return pixels;
}

/* ---- bark, foliage, masonry, planks, metal ---- */

uint8_t* texture_bark(unsigned seed) {
  /* Furrowed bark: long ridges with dark splits between, lichen patches. */
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      float warp = perlin_xy((float)x, (float)y, 6, 4, seed + 3) - 0.5f;
      float ridges = perlin_xy((float)x + warp * 60.0f, (float)y, 28, 3, seed);
      float split = smoothstep(0.55f, 0.35f, ridges);
      float fine = perlin_xy((float)x, (float)y, 120, 24, seed + 5);
      float k = 0.55f + 0.7f * ridges;
      k *= 0.85f + 0.3f * fine;
      k *= 1.0f - 0.55f * split;
      float lichen =
          smoothstep(0.7f, 0.85f, fbm((float)x, (float)y, 8, 3, seed + 11));
      float r = 0.30f * k, g = 0.23f * k, b = 0.17f * k;
      r = mix(r, 0.34f, lichen * 0.35f);
      g = mix(g, 0.38f, lichen * 0.35f);
      b = mix(b, 0.2f, lichen * 0.35f);
      put(pixels, x, y, r, g, b);
    }
  return pixels;
}

uint8_t* texture_foliage(unsigned seed) {
  /* A mass of leaves: each its own green, edges catching light, gaps dark,
   * clumps shadowed, the odd yellowing leaf. */
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      float id, second;
      float d = worley((float)x, (float)y, 120, seed, &id, &second);
      float clump = fbm((float)x, (float)y, 5, 4, seed + 7);
      float leaf = smoothstep(0.62f, 0.38f, d);
      float edge = smoothstep(0.38f, 0.55f, d) * leaf;
      float hue = id;
      float r = mix(0.10f, 0.22f, hue), g = mix(0.24f, 0.42f, hue),
            b = mix(0.05f, 0.10f, hue);
      if (id > 0.93f) {
        r = 0.42f;
        g = 0.44f;
        b = 0.1f;
      }
      float shade_k = 0.55f + 0.75f * clump;
      shade_k *= 0.75f + 0.5f * (1.0f - d);
      shade_k *= 1.0f + 0.25f * edge;
      float gap = 1.0f - leaf;
      shade_k *= 1.0f - 0.7f * gap;
      put(pixels, x, y, r * shade_k, g * shade_k, b * shade_k);
    }
  return pixels;
}

uint8_t* texture_stone(unsigned seed) {
  /* Rubble masonry: irregular stones, buff to grey, in lime mortar. */
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      float id, second;
      float d = worley((float)x, (float)y, 9, seed, &id, &second);
      float gap = second - d;
      bool mortar = gap < 0.085f;
      float mottle = fbm((float)x, (float)y, 16, 4, seed + 3);
      float fine = noise((unsigned)x, (unsigned)y, seed + 5);
      if (mortar) {
        float m = 0.42f + 0.1f * mottle;
        put(pixels, x, y, m, m * 0.97f, m * 0.9f);
        continue;
      }
      float bevel = 0.7f + 0.3f * smoothstep(0.085f, 0.22f, gap);
      float r = mix(0.56f, 0.46f, id), g = mix(0.5f, 0.44f, id),
            b = mix(0.4f, 0.4f, id);
      if (id > 0.75f) {
        r = 0.5f;
        g = 0.42f;
        b = 0.34f;
      }
      float k = bevel * (0.8f + 0.4f * mottle) * (0.92f + 0.16f * fine);
      put(pixels, x, y, r * k, g * k, b * k);
    }
  return pixels;
}

uint8_t* texture_planks(unsigned seed) {
  /* Weathered boards, upright, each its own silver-brown with grain, knots
   * and dark gaps. */
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  const int board = 64;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      int b_index = x / board;
      float bx = (float)(x % board);
      unsigned id = hash((unsigned)b_index, 1, seed);
      float v = (float)(id & 0xff) / 255.0f;
      float grain =
          perlin_xy((float)x, (float)y + (float)(id % 50), 128, 5, seed + 3);
      float rings = perlin_xy((float)x, (float)y, 40, 2, seed + 9);
      float k = 0.8f + 0.5f * grain + 0.15f * rings;
      float r = mix(0.40f, 0.30f, v), g = mix(0.32f, 0.26f, v),
            b = mix(0.24f, 0.2f, v);
      /* Silvered by weather in patches. */
      float silver =
          smoothstep(0.5f, 0.8f, fbm((float)x, (float)y, 5, 3, seed + 13));
      r = mix(r, 0.45f, silver * 0.5f), g = mix(g, 0.43f, silver * 0.5f),
      b = mix(b, 0.4f, silver * 0.5f);
      float edge = fminf(bx, (float)board - bx);
      if (edge < 2.0f)
        k *= 0.25f;
      else if (edge < 5.0f)
        k *= 0.8f;
      /* A knot now and then. */
      float knot_x = 14.0f + (float)((id >> 8) % 36),
            knot_y = (float)((id >> 14) % S);
      float kd = hypotf(bx - knot_x,
                        fmodf((float)y - knot_y + (float)S, (float)S) - 0.0f);
      if (kd < 7.0f && (id % 3) == 0) k *= 0.55f + 0.4f * kd / 7.0f;
      put(pixels, x, y, r * k, g * k, b * k);
    }
  return pixels;
}

uint8_t* texture_metal(unsigned seed) {
  /* Painted steel: a smooth dark finish with fine scratches, dull patches and
   * a little rust weeping from the lower edge. */
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      float broad = fbm((float)x, (float)y, 4, 3, seed);
      float scratch = perlin_xy((float)x, (float)y, 6, 300, seed + 3);
      float k = 0.9f + 0.2f * broad;
      k *= 1.0f - 0.25f * smoothstep(0.72f, 0.9f, scratch);
      float r = 0.20f * k, g = 0.21f * k, b = 0.23f * k;
      float rust =
          smoothstep(0.7f, 0.85f, fbm((float)x, (float)y, 10, 4, seed + 7)) *
          (0.3f + 0.7f * (1.0f - (float)y / (float)S));
      r = mix(r, 0.34f, rust * 0.7f), g = mix(g, 0.17f, rust * 0.7f),
      b = mix(b, 0.09f, rust * 0.7f);
      put(pixels, x, y, r, g, b);
    }
  return pixels;
}

uint8_t* texture_checker(void) {
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  for (int y = 0; y < S; y++)
    for (int x = 0; x < S; x++) {
      bool dark = ((x / 64) + (y / 64)) % 2 == 0;
      put(pixels, x, y, dark ? 0.35f : 0.75f, dark ? 0.35f : 0.75f,
          dark ? 0.35f : 0.75f);
    }
  return pixels;
}

/* The textures are the same every time, so each is painted once per run and
 * kept (in static storage: about 40 MB) for the next stage to upload from. */
#define TEXTURE_SLOTS 40
static uint8_t cache[TEXTURE_SLOTS][S * S * 4];
static bool cached[TEXTURE_SLOTS];

static texture_handle_t upload_slot(renderer_t* renderer, int slot,
                                    uint8_t* (*make)(unsigned), unsigned seed) {
  texture_handle_t none = {0};
  if (!cached[slot]) {
    uint8_t* pixels = make(seed);
    if (!pixels) return none;
    memcpy(cache[slot], pixels, sizeof(cache[slot]));
    free(pixels);
    cached[slot] = true;
  }
  return renderer_upload_surface_texture(renderer, S, S, cache[slot]);
}

static uint8_t* make_checker(unsigned seed) {
  (void)seed;
  return texture_checker();
}

/* Facades take their window columns too: one function per count. */
static uint8_t* make_facade_one(unsigned seed) {
  return texture_facade(seed, 1, 1);
}
static uint8_t* make_facade_two(unsigned seed) {
  return texture_facade(seed, 1, 2);
}
static uint8_t* make_fresco(unsigned seed) {
  return texture_facade_fresco(seed);
}
static uint8_t* make_brick(unsigned seed) { return texture_facade_brick(seed); }

static const char* const sign_words[SIGN_WORDS] = {
    "CINEMA",   "GARAGE", "SHOP",    "BAR",    "CAFE",      "PIZZERIA",
    "FARMACIA", "HOTEL",  "BANK",    "BOOKS",  "TRATTORIA", "GELATERIA",
    "BAKERY",   "MARKET", "TOBACCO", "FLOWERS"};

const char* sign_word(int index) {
  return sign_words[((index % SIGN_WORDS) + SIGN_WORDS) % SIGN_WORDS];
}

/* Boards and letters: dark green with cream, red with white, navy with
 * yellow, black with gold, cream with dark red, slate with white. */
uint8_t* texture_signs(unsigned seed) {
  static const float boards[][3] = {
      {0.07f, 0.24f, 0.14f}, {0.55f, 0.07f, 0.07f}, {0.07f, 0.12f, 0.32f},
      {0.05f, 0.05f, 0.06f}, {0.86f, 0.80f, 0.64f}, {0.22f, 0.25f, 0.30f}};
  static const float letters[][3] = {
      {0.92f, 0.88f, 0.70f}, {0.96f, 0.95f, 0.90f}, {0.95f, 0.80f, 0.22f},
      {0.85f, 0.68f, 0.25f}, {0.45f, 0.07f, 0.07f}, {0.95f, 0.95f, 0.92f}};
  uint8_t* pixels = allocate();
  if (!pixels) return NULL;
  const int cell_w = S / 2, cell_h = S / 8;
  for (int word = 0; word < SIGN_WORDS; word++) {
    int ox = (word % 2) * cell_w, oy = (word / 2) * cell_h;
    int scheme = (int)((seed + (unsigned)word * 5u) % 6u);
    const float* board = boards[scheme];
    const float* ink = letters[scheme];
    const char* text = sign_words[word];
    int length = (int)strlen(text);
    int scale = length <= 7 ? 5 : 4;
    int pitch = 6 * scale;
    int text_w = length * pitch - scale;
    int tx = ox + (cell_w - text_w) / 2;
    int ty = oy + (cell_h - 7 * scale) / 2;
    for (int y = 0; y < cell_h; y++)
      for (int x = 0; x < cell_w; x++) {
        float grain = 0.93f + 0.14f * noise((unsigned)(ox + x),
                                            (unsigned)(oy + y), seed + 3);
        float wear = fbm((float)(ox + x), (float)(oy + y), 24, 3, seed + 7);
        float k = grain * (0.9f + 0.2f * wear);
        float r = board[0] * k, g = board[1] * k, b = board[2] * k;
        /* A raised rim round the board: light on top, dark under. */
        int edge = x < y ? x : y;
        int far_x = cell_w - 1 - x, far_y = cell_h - 1 - y;
        int rim = edge < far_x ? edge : far_x;
        if (far_y < rim) rim = far_y;
        if (rim < 3) {
          float light = (y < 3 || x < 3) ? 1.5f : 0.55f;
          r *= light, g *= light, b *= light;
          r = mix(r, ink[0], 0.35f), g = mix(g, ink[1], 0.35f),
          b = mix(b, ink[2], 0.35f);
        }
        /* Lettering, with a dark shadow one pixel down and right. */
        int lx = x + ox - tx, ly = y + oy - ty;
        bool lit = false, shadow = false;
        for (int pass = 0; pass < 2; pass++) {
          int qx = lx - pass * 2, qy = ly - pass * 2;
          if (qx < 0 || qy < 0) continue;
          int row = qy / scale;
          int letter = qx / pitch, inside = qx % pitch;
          if (letter < length && inside < 5 * scale && row < 7 &&
              font5x7_pixel(text[letter], row, inside / scale)) {
            if (pass == 0)
              lit = true;
            else
              shadow = true;
          }
        }
        if (lit) {
          float paint = 0.9f + 0.1f * noise((unsigned)(ox + x),
                                            (unsigned)(oy + y), seed + 9);
          r = ink[0] * paint, g = ink[1] * paint, b = ink[2] * paint;
        } else if (shadow) {
          r *= 0.35f, g *= 0.35f, b *= 0.35f;
        }
        put(pixels, ox + x, oy + y, r, g, b);
      }
  }
  return pixels;
}

texture_set_t texture_set_upload(renderer_t* renderer) {
  texture_set_t set;
  int slot = 0;
  set.asphalt = upload_slot(renderer, slot++, texture_asphalt, 1);
  set.road = upload_slot(renderer, slot++, texture_road, 2);
  set.concrete = upload_slot(renderer, slot++, texture_concrete, 3);
  set.grass = upload_slot(renderer, slot++, texture_grass, 4);
  set.earth = upload_slot(renderer, slot++, texture_earth, 17);
  static const unsigned facade_seeds[CITY_FACADE_STYLES] = {
      36, 49, 62, 75, 88, 101, 114, 127, 140, 153, 166, 179};
  for (int i = 0; i < CITY_FACADE_STYLES; i++)
    set.facades[i] = upload_slot(
        renderer, slot++, (i % 3) == 2 ? make_facade_two : make_facade_one,
        facade_seeds[i]);
  set.facade = set.facades[0];
  set.roof = upload_slot(renderer, slot++, texture_roof, 6);
  set.facade_fresco = upload_slot(renderer, slot++, make_fresco, 5);
  set.facade_brick = upload_slot(renderer, slot++, make_brick, 23);
  set.paving = upload_slot(renderer, slot++, texture_paving, 53);
  set.door = upload_slot(renderer, slot++, texture_door, 59);
  set.shopfront = upload_slot(renderer, slot++, texture_shopfront, 31);
  set.roof_slate = upload_slot(renderer, slot++, texture_roof_slate, 8);
  set.roof_flat = upload_slot(renderer, slot++, texture_roof_flat, 9);
  set.bark = upload_slot(renderer, slot++, texture_bark, 71);
  set.foliage = upload_slot(renderer, slot++, texture_foliage, 72);
  set.stone = upload_slot(renderer, slot++, texture_stone, 73);
  set.planks = upload_slot(renderer, slot++, texture_planks, 74);
  set.metal = upload_slot(renderer, slot++, texture_metal, 75);
  set.signs = upload_slot(renderer, slot++, texture_signs, 81);
  set.checker = upload_slot(renderer, slot++, make_checker, 0);
  return set;
}
