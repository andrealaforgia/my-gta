/**
 * @file vehicle.h
 * @brief A car: a rigid chassis in the physics world on four raycast
 * wheels with spring-damper suspension, a slip-based tyre model inside a
 * friction circle, an engine torque curve, an automatic gearbox and
 * reverse. Every tunable lives in vehicle_params_t, loaded from a
 * key = value file so handling changes without recompiling.
 *
 * Call vehicle_update before each physics step; it applies forces to the
 * body for that step and reads the previous step's state.
 */
#ifndef ENGINE_VEHICLE_VEHICLE_H_
#define ENGINE_VEHICLE_VEHICLE_H_

#include <stdbool.h>

#include "input.h"
#include "physics_world.h"
#include "transform.h"

#define VEHICLE_WHEEL_COUNT 4
#define VEHICLE_MAX_GEARS 8
#define VEHICLE_MAX_TORQUE_POINTS 16
#define VEHICLE_GEAR_REVERSE (-1)
#define VEHICLE_NAME_LENGTH 32

typedef enum {
  VEHICLE_DRIVE_FRONT,
  VEHICLE_DRIVE_REAR,
  VEHICLE_DRIVE_ALL
} vehicle_drive_t;

typedef struct {
  float rpm;
  float torque; /* Nm */
} torque_point_t;

typedef struct {
  char name[VEHICLE_NAME_LENGTH];
  float mass;
  vec3_t chassis_half_extents;
  float track_width;
  float wheelbase;
  float wheel_attach_height; /* local y of the suspension top */
  float wheel_radius;
  float wheel_width;
  float suspension_rest_length;
  float spring_stiffness;   /* N/m */
  float damper_compression; /* Ns/m */
  float damper_rebound;     /* Ns/m */
  float tyre_friction;      /* peak coefficient on a friction 1.0 surface */
  /* Lateral force per radian of slip, as a fraction of load. */
  float tyre_lateral_stiffness;
  float rolling_resistance; /* fraction of load */
  /* How far above the tyre contact the horizontal tyre forces act on the
   * body: the suspension's roll centre. The lower the car's real centre of
   * mass sits relative to it, the less the body leans in a corner. */
  float roll_centre_lift;
  float drag_coefficient; /* N per (m/s)^2 */
  float max_steer_angle;  /* radians at full lock */
  float steering_rate;
  float steering_half_lock_speed;
  float brake_force;     /* N, whole car */
  float handbrake_force; /* N, rear axle */
  float handbrake_grip_factor;
  float engine_idle_rpm;
  float engine_max_rpm;
  float engine_braking; /* fraction of peak torque at max rpm */
  torque_point_t torque_curve[VEHICLE_MAX_TORQUE_POINTS];
  int torque_point_count;
  float gear_ratios[VEHICLE_MAX_GEARS];
  int gear_count;
  float reverse_ratio;
  float final_drive;
  float drivetrain_efficiency;
  float shift_up_rpm;
  float shift_down_rpm;
  float shift_time;
  vehicle_drive_t drive;
} vehicle_params_t;

typedef struct {
  vec3_t attach_local;
  bool steerable;
  bool driven;
  bool handbraked;
  bool grounded;
  float compression;      /* m */
  float compression_rate; /* m/s */
  float load;             /* N through the spring */
  vec3_t contact_point;
  vec3_t contact_normal;
  float surface_friction;
  float steer_angle; /* radians, positive turns left */
  float spin;        /* rad/s */
  float rotation;    /* accumulated, radians */
  float slip_angle;  /* radians */
  float grip_use;    /* 0..1, fraction of the friction circle in use */
  bool skidding;
  vec3_t hub_position; /* world */
} wheel_state_t;

typedef struct {
  vehicle_params_t params;
  body_handle_t body;
  wheel_state_t wheels[VEHICLE_WHEEL_COUNT];
  int gear; /* 1.. forward, VEHICLE_GEAR_REVERSE */
  float rpm;
  float throttle;
  float brake;
  float steering; /* filtered, -1..1 */
  bool handbrake;
  float shift_timer;
  steering_filter_t steering_filter;
  vec3_t spawn_position;
  quat_t spawn_rotation;
  float forward_speed;  /* m/s along the chassis forward axis */
  float skid_intensity; /* 0..1 */
  float impact_impulse; /* largest chassis contact impulse this step */
  float damage;         /* 0 new to 1 wrecked: grows with hard impacts */
} vehicle_t;

vehicle_params_t vehicle_params_default(void);
bool vehicle_params_load(vehicle_params_t* params, const char* path);
float vehicle_engine_torque(const vehicle_params_t* params, float rpm);

bool vehicle_create(vehicle_t* vehicle, physics_world_t* world,
                    const vehicle_params_t* params, vec3_t position,
                    quat_t rotation);
void vehicle_destroy(vehicle_t* vehicle, physics_world_t* world);
void vehicle_set_spawn(vehicle_t* vehicle, vec3_t position, quat_t rotation);
void vehicle_reset(vehicle_t* vehicle, physics_world_t* world);
void vehicle_update(vehicle_t* vehicle, physics_world_t* world,
                    driving_input_t input, float dt);

float vehicle_forward_speed(const vehicle_t* vehicle);
/* Engine power left after damage: 1 when sound, down to 0.55. */
float vehicle_power_factor(const vehicle_t* vehicle);
float vehicle_speed_kmh(const vehicle_t* vehicle);
float vehicle_rpm(const vehicle_t* vehicle);
int vehicle_gear(const vehicle_t* vehicle);
float vehicle_skid_intensity(const vehicle_t* vehicle);
/* Where to draw wheel `index` given the (interpolated) chassis transform. */
transform_t vehicle_wheel_transform(const vehicle_t* vehicle,
                                    transform_t chassis, int index);

#endif  // ENGINE_VEHICLE_VEHICLE_H_
