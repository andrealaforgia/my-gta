/**
 * @file driving_stage.h
 * @brief The driving prototype: one scene, one car, chase camera, HUD,
 * audio and debug overlays. Composes engine services; implements none.
 */
#ifndef GAME_SRC_STAGE_DRIVING_STAGE_H_
#define GAME_SRC_STAGE_DRIVING_STAGE_H_

#include <stdbool.h>

#include "audio_device.h"
#include "audio_mixer.h"
#include "car_visual.h"
#include "chase_camera.h"
#include "fleet.h"
#include "frame_clock.h"
#include "parked_cars.h"
#include "pedestrians.h"
#include "physics_world.h"
#include "renderer.h"
#include "scene.h"
#include "skid_marks.h"
#include "textures.h"
#include "traffic.h"
#include "vehicle.h"
#include "window.h"
#include "world.h"

typedef enum { SCENE_TEST_TRACK, SCENE_CITY } scene_kind_t;

typedef struct {
  scene_kind_t scene;
  const char* city_package; /* for SCENE_CITY */
  const char* vehicle_file;
  bool show_debug;
  bool muted;
  float start_hour;   /* time of day at the start, 0..24 */
  float hour_seconds; /* real seconds per game hour; 0 freezes the clock */
  int traffic_cars;   /* moving cars about the city; parked cars always stand */
  int pedestrians;    /* people on the pavements */
  float rain;         /* 0 clear to 1 pouring at the start */
} stage_options_t;

typedef struct {
  window_t* window;
  renderer_t* renderer;
  audio_device_t* audio;
  physics_world_t* physics;
  world_t world;
  texture_set_t textures;
  scene_t scene;
  vehicle_t vehicle;
  car_visual_t car_visual;
  chase_camera_t camera;
  frame_clock_t clock;
  bool paused;
  float rain; /* 0..1, eased towards rain_target */
  float rain_target;
  float rain_clock;   /* animates the falling drops */
  float shake; /* 0..1: the view jolts after a hard impact and settles */
  float signal_clock; /* seconds on the traffic signals' cycle */
  float hour;         /* time of day, 0..24 */
  float hour_seconds;
  skid_marks_t skid_marks;
  mesh_handle_t skid_strip;
  fleet_t fleet;
  bool has_fleet;
  parked_cars_t parked;
  traffic_t* traffic;
  pedestrians_t* people;
  vec3_t walkers[64]; /* who is crossing a street now, for the traffic */
  pedestrian_meshes_t people_meshes;
  bool horn; /* the horn is being held */
  int vehicle_index;
  int view_mode; /* 0 chase, 1 bonnet, 2 cockpit */
  bool map_open; /* M: the city map over a paused game */
  texture_handle_t map_texture;
  bool show_debug;
  bool quit;
  float simulated_seconds;
  double frame_ms;
  double physics_ms;
  double fps;
  double fps_accumulator;
  int fps_frames;
  int last_drawn;
} driving_stage_t;

bool driving_stage_init(driving_stage_t* stage, window_t* window,
                        renderer_t* renderer, audio_device_t* audio,
                        stage_options_t options);
void driving_stage_destroy(driving_stage_t* stage);
/* What the car should sound like now: silent while paused or on the map. */
audio_state_t driving_stage_audio_state(const driving_stage_t* stage);
/* Swaps the player's car for the next of hatchback, saloon, van and city
 * car, on the spot, keeping its heading. */
void driving_stage_next_vehicle(driving_stage_t* stage);
/* Rain from 0 to 1 now: eased towards its target a little each frame. */
void driving_stage_set_rain(driving_stage_t* stage, float rain);
/* The steering wheel's angle: one and a half turns each way at full lock. */
float driving_stage_wheel_angle(const driving_stage_t* stage);
/* The camera for the current view: behind the car, over the bonnet or
 * from the driver's seat. */
render_camera_t driving_stage_camera(const driving_stage_t* stage);
/* One fixed simulation step with the given input. */
void driving_stage_step(driving_stage_t* stage, driving_input_t input,
                        float dt);
/* Advance the clock by one frame's wall time, simulate, update the camera. */
void driving_stage_advance(driving_stage_t* stage, driving_input_t input,
                           double frame_seconds);
void driving_stage_render(driving_stage_t* stage);
/* Poll the window and run until quit. */
void driving_stage_run(driving_stage_t* stage);

#endif  // GAME_SRC_STAGE_DRIVING_STAGE_H_
