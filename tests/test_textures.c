/* Procedural textures: the properties that make them read as what they are. */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "test_groups.h"
#include "test_support.h"
#include "textures.h"

static float luminance_at(const uint8_t* pixels, int x, int y) {
  const uint8_t* p = pixels + (y * TEXTURE_SIZE + x) * 4;
  return (0.3f * p[0] + 0.6f * p[1] + 0.1f * p[2]) / 255.0f;
}

static float mean_over(const uint8_t* pixels, float u0, float u1) {
  double sum = 0;
  int count = 0;
  for (int y = 0; y < TEXTURE_SIZE; y++)
    for (int x = (int)(u0 * TEXTURE_SIZE); x < (int)(u1 * TEXTURE_SIZE); x++) {
      sum += luminance_at(pixels, x, y);
      count++;
    }
  return (float)(sum / count);
}

static void roads_show_wheel_tracks_and_lane_markings(void) {
  uint8_t* road = texture_road(2);
  /* Two lanes: tracks at an eighth and three eighths across each. */
  float track = mean_over(road, 0.115f, 0.135f);
  float between = mean_over(road, 0.2f, 0.3f);
  assert(track < between * 0.94f);
  /* The centre line is still the brightest thing on it. */
  float centre = 0;
  for (int y = 0; y < 32; y++)
    centre += luminance_at(road, TEXTURE_SIZE / 2, y);
  assert(centre / 32 > 2.0f * between);
  /* Patches and cracks: the surface is not uniform along its length. */
  float lowest = 1, highest = 0;
  for (int row = 0; row < 8; row++) {
    double sum = 0;
    for (int y = row * 32; y < row * 32 + 32; y++)
      for (int x = (int)(0.2f * TEXTURE_SIZE); x < (int)(0.3f * TEXTURE_SIZE);
           x++)
        sum += luminance_at(road, x, y);
    float mean = (float)(sum / (32.0 * (0.1f * TEXTURE_SIZE)));
    lowest = fminf(lowest, mean), highest = fmaxf(highest, mean);
  }
  assert(highest > lowest * 1.05f);
  free(road);
}

/* Mean absolute difference between neighbouring pixels across a seam, over
 * all rows, as a share of the same in the interior: a texture that tiles has
 * no visible seam, so the two are alike. */
static float seam_ratio(const uint8_t* pixels, bool vertical) {
  double edge = 0, inside = 0;
  for (int i = 0; i < TEXTURE_SIZE; i++) {
    float a = vertical ? luminance_at(pixels, i, TEXTURE_SIZE - 1)
                       : luminance_at(pixels, TEXTURE_SIZE - 1, i);
    float b =
        vertical ? luminance_at(pixels, i, 0) : luminance_at(pixels, 0, i);
    float c =
        vertical ? luminance_at(pixels, i, 200) : luminance_at(pixels, 200, i);
    float d =
        vertical ? luminance_at(pixels, i, 201) : luminance_at(pixels, 201, i);
    edge += fabsf(a - b);
    inside += fabsf(c - d);
  }
  return (float)(edge / (inside + 1e-6));
}

static void surface_textures_tile_without_a_seam(void) {
  /* The noise-like ones: tiles, boards and courses have edges by design. */
  uint8_t* set[] = {texture_asphalt(1),  texture_concrete(3),  texture_grass(4),
                    texture_earth(17),   texture_roof_flat(9), texture_bark(71),
                    texture_foliage(72), texture_metal(75)};
  for (size_t i = 0; i < sizeof(set) / sizeof(set[0]); i++) {
    assert(set[i]);
    assert(seam_ratio(set[i], true) < 3.0f && seam_ratio(set[i], false) < 3.0f);
    free(set[i]);
  }
}

static void facades_have_glass_that_the_renderer_can_mirror_and_light(void) {
  uint8_t* facade = texture_facade(36, 1, 1);
  int glass = 0, wall = 0;
  for (int i = 0; i < TEXTURE_SIZE * TEXTURE_SIZE; i++)
    if (facade[i * 4 + 3] == 0)
      glass++;
    else
      wall++;
  /* Some glass, but mostly wall. */
  assert(glass > TEXTURE_SIZE * TEXTURE_SIZE / 30 && wall > glass * 2);
  free(facade);
  /* Brick has mortar joints: lighter lines between darker bricks. */
  uint8_t* brick = texture_facade_brick(23);
  float lowest = 1, highest = 0;
  for (int x = 0; x < 90; x++) {
    float l = luminance_at(brick, x, 40);
    lowest = fminf(lowest, l), highest = fmaxf(highest, l);
  }
  assert(highest > lowest * 1.4f);
  free(brick);
}

static void shop_signs_hold_readable_words_on_painted_boards(void) {
  uint8_t* signs = texture_signs(81);
  assert(signs);
  assert(strcmp(sign_word(0), "CINEMA") == 0 &&
         strcmp(sign_word(1), "GARAGE") == 0 &&
         strcmp(sign_word(2), "SHOP") == 0);
  /* Every board has lettering that stands out from it: the brightest and
   * darkest pixels of each differ by a good deal, and most of the board is
   * plain paint. */
  for (int word = 0; word < SIGN_WORDS; word++) {
    int ox = (word % 2) * (TEXTURE_SIZE / 2),
        oy = (word / 2) * (TEXTURE_SIZE / 8);
    float lowest = 1, highest = 0;
    int lettering = 0, pixels = 0;
    float board = luminance_at(signs, ox + 10, oy + 10);
    for (int y = 4; y < TEXTURE_SIZE / 8 - 4; y++)
      for (int x = 4; x < TEXTURE_SIZE / 2 - 4; x++) {
        float l = luminance_at(signs, ox + x, oy + y);
        lowest = fminf(lowest, l), highest = fmaxf(highest, l);
        if (fabsf(l - board) > 0.25f) lettering++;
        pixels++;
      }
    assert(highest - lowest > 0.4f);
    assert(lettering > pixels / 25 && lettering < pixels / 2);
  }
  free(signs);
}

void run_textures_tests(void) {
  RUN(shop_signs_hold_readable_words_on_painted_boards);
  RUN(roads_show_wheel_tracks_and_lane_markings);
  RUN(surface_textures_tile_without_a_seam);
  RUN(facades_have_glass_that_the_renderer_can_mirror_and_light);
}
