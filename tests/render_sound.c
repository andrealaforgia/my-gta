/* Renders ten seconds of a standing start to a WAV file: idle, full
 * throttle through the gears with wind rising, a skid, an impact, then
 * lift off. No audio device needed, so the result can be judged by ear
 * without launching the game. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio_mixer.h"

#define RATE 48000
#define SECONDS 10

static void write_u32(FILE* f, uint32_t v) { fwrite(&v, 4, 1, f); }
static void write_u16(FILE* f, uint16_t v) { fwrite(&v, 2, 1, f); }

int main(int argc, char* argv[]) {
  const char* path = argc > 1 ? argv[1] : "engine.wav";
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  state.engine_running = true;
  int total = RATE * SECONDS;
  int16_t* pcm = malloc(total * sizeof(int16_t));
  float block[RATE / 100];
  if (!pcm) return EXIT_FAILURE;
  for (int i = 0; i < total; i += RATE / 100) {
    float t = (float)i / RATE;
    /* A rough gearbox: revs climb and drop back three times. */
    float phase = t < 1.5f ? 0 : fmodf((t - 1.5f) / 2.2f, 1.0f);
    state.throttle = t < 1.5f ? 0.0f : (t < 8.5f ? 1.0f : 0.0f);
    state.rpm =
        t < 1.5f ? 900
                 : (t < 8.5f ? 1800 + phase * 4300 : 2500 - (t - 8.5f) * 800);
    state.speed_mps =
        t < 1.5f ? 0
                 : (t < 8.5f ? (t - 1.5f) * 4.0f : 28.0f - (t - 8.5f) * 6.0f);
    state.skid = (t > 1.6f && t < 2.2f) ? 0.8f : 0.0f;
    audio_mixer_set_state(&mixer, state);
    if (i == RATE * 6) audio_mixer_trigger_impact(&mixer, 0.7f);
    audio_mixer_render(&mixer, block, RATE / 100);
    for (int k = 0; k < RATE / 100 && i + k < total; k++) {
      float v = block[k];
      pcm[i + k] = (int16_t)(v * 32000.0f);
    }
  }
  FILE* f = fopen(path, "wb");
  if (!f) return EXIT_FAILURE;
  uint32_t data_bytes = (uint32_t)total * 2;
  fwrite("RIFF", 1, 4, f);
  write_u32(f, 36 + data_bytes);
  fwrite("WAVEfmt ", 1, 8, f);
  write_u32(f, 16);
  write_u16(f, 1);
  write_u16(f, 1);
  write_u32(f, RATE);
  write_u32(f, RATE * 2);
  write_u16(f, 2);
  write_u16(f, 16);
  fwrite("data", 1, 4, f);
  write_u32(f, data_bytes);
  fwrite(pcm, 2, (size_t)total, f);
  fclose(f);
  free(pcm);
  printf("wrote %s (%d s)\n", path, SECONDS);
  return EXIT_SUCCESS;
}
