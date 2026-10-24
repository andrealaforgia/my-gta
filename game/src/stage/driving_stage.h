/**
 * @file driving_stage.h
 * @brief The driving prototype: one scene, one car, chase camera, HUD,
 * audio and debug overlays. Composes engine services; implements none.
 */
#ifndef GAME_SRC_STAGE_DRIVING_STAGE_H_
#define GAME_SRC_STAGE_DRIVING_STAGE_H_

#include <stdbool.h>

#include "aircraft.h"
#include "aircraft_visual.h"
#include "audio_device.h"
#include "audio_mixer.h"
#include "car_visual.h"
#include "chase_camera.h"
#include "fleet.h"
#include "frame_clock.h"
#include "guide_hud.h"
#include "navigation.h"
#include "parked_cars.h"
#include "pedestrians.h"
#include "physics_world.h"
#include "renderer.h"
#include "scene.h"
#include "skid_marks.h"
#include "textures.h"
#include "traffic.h"
#include "vehicle.h"
#include "walker.h"
#include "wanted.h"
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
  bool has_menu;      /* Backspace may go back to the start menu */
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
  float burning; /* seconds the car has been on fire */
  float blast;   /* seconds since it blew up; 0 while it has not */
  vec3_t blast_at;
  bool blast_kills;    /* whether the player is close enough to die in it */
  bool blast_car;      /* the player's car blew up, not an aircraft */
  bool wrecked;        /* the car has blown up and is no use any more */
  float entering;      /* seconds into getting into a car; 0 when not */
  vec3_t entry_start;  /* where the walker set off from */
  float shot_cooldown; /* seconds until the pistol can fire again */
  float shot_flash;    /* seconds the last shot's tracer is shown */
  vec3_t shot_from, shot_to;
  int people_struck;  /* people the car ran into this step */
  int radio_station;  /* 0 off .. RADIO_STATIONS */
  float radio_banner; /* seconds left to show the station's name */
  float rain_clock;   /* animates the falling drops */
  char street[48];    /* the street under the car, for the screen */
  float street_clock; /* seconds since it was last looked up */
  float shake;        /* 0..1: the view jolts after a hard impact and settles */
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
  bool city_rules;  /* police, satnav and fines, in a city */
  guide_t guide;    /* the way to the airport */
  bool has_figures; /* meshes for drawing people, so the player can walk */
  bool on_foot;     /* out of the car, walking (or flying: see piloting) */
  aircraft_visual_t air_visual;
  bool has_air;
  aircraft_t aircraft[CITY_PACKAGE_MAX_AIRCRAFT];
  aircraft_t aircraft_home[CITY_PACKAGE_MAX_AIRCRAFT]; /* where it stands */
  size_t aircraft_count;
  int piloting; /* 0 on the ground, else the aircraft flown, from 1 */
  struct {
    bool active;     /* the player is coming down on a parachute */
    bool opened;     /* the canopy has filled (its thump has sounded) */
    float time;      /* seconds since they jumped */
    float height;    /* metres above the ground or roof below */
    float bank;      /* how far they lean into a turn, -1 to 1 */
    vec3_t velocity; /* metres a second */
  } chute;
  chase_camera_params_t saved_camera_params; /* kept while flying */
  walker_t walker;
  int walker_outfit;
  chase_camera_params_t car_camera_params; /* kept while on foot */
  wanted_t wanted;
  char notice[40];   /* a line across the screen for a moment */
  float notice_time; /* seconds it has left */
  nav_t* nav;
  float satnav[2 * 1024]; /* the way to the airport, x, z pairs */
  size_t satnav_count;
  float satnav_clock; /* seconds since the way was last worked out */
  bool horn;          /* the horn is being held */
  int vehicle_index;
  int view_mode; /* 0 chase, 1 bonnet, 2 cockpit */
  bool map_open; /* M: the city map over a paused game */
  texture_handle_t map_texture;
  bool show_debug;
  bool has_menu;
  bool back_to_menu; /* the player asked for the start menu */
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
/* F: out of the car if it is nearly stopped, or, on foot, into the car left
 * behind or the nearest other car (standing or driving), which is stolen. */
/* Back to the start, mended, with everything under way stopped (R). */
void driving_stage_reset(driving_stage_t* stage);
void driving_stage_interact(driving_stage_t* stage);
/* Fires the pistol, when on foot: the first thing in the way is hit. A
 * person who goes down makes it a crime. Returns true if a shot was fired. */
bool driving_stage_fire(driving_stage_t* stage);
/* A line across the screen for a moment. */
void driving_stage_say(driving_stage_t* stage, const char* text);
/* The player did something the law minds (stealing an aircraft). */
void driving_stage_commit_crime(driving_stage_t* stage);
/* The player's vehicle blows up at `at`; the player wakes at the start. */
void driving_stage_crash_at(driving_stage_t* stage, vec3_t at);
/* Makes the player's vehicle fleet variant `index`, at a place and facing. */
bool driving_stage_set_vehicle(driving_stage_t* stage, int index,
                               vec3_t position, quat_t rotation);
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
