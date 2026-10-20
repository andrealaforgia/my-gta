# Vehicle physics

The car is one rigid box body in the physics world, with four wheels
simulated as rays. Every parameter is in `assets/vehicles/<car>.cfg`
and read at start, so handling is tuned without recompiling.

## Chassis

Mass, box half extents and the inertia of a uniform box. Air drag
`drag_coefficient x v^2` at the centre. The body's corners contact
static geometry (buildings, barriers, the ground when the suspension
bottoms out) with Coulomb friction and low restitution.

## Suspension

Each wheel casts a ray from its attach point down the chassis up axis,
`suspension_rest_length + wheel_radius` long. Compression is what is left
of the rest length; the spring force is `spring_stiffness x compression
+ damper x compression rate` with separate compression and rebound
damping, never negative, applied at the attach point. Load for the tyre
is that spring force.

## Tyres

Contact velocity is split into forward and sideways components on the
contact plane, with the front wheels yawed by the steering angle.

- Longitudinal force: drive force from the engine minus braking, rolling
  resistance and handbrake, with braking capped at what would stop the
  wheel's share of the car in one step so it never reverses.
- Lateral force: `-tyre_lateral_stiffness x slip angle x load`, capped at
  what would cancel the sideways speed in one step.
- The two are scaled together into a friction circle of radius
  `tyre_friction x surface friction x load`. Exceeding it, or a slip
  angle over 0.12 rad at speed, marks the wheel as skidding, which feeds
  the audio and the debug overlay. The handbrake locks the rear wheels
  and multiplies their grip by `handbrake_grip_factor`.

## Drivetrain

Engine speed follows the driven wheels through the gear and final drive;
below idle the clutch slips. Torque comes from the interpolated
`torque_curve`, times throttle, zero above `engine_max_rpm`. Off throttle
there is engine braking growing with revs. An automatic gearbox shifts up
at `shift_up_rpm` and down at `shift_down_rpm` with a `shift_time` cut.
Holding the brake under 0.4 m/s selects reverse; in reverse the brake
pedal accelerates backwards and the throttle brakes.

## Steering

Keyboard steering is a demand filtered towards its target at
`steering_rate` per second; the reachable lock halves at
`steering_half_lock_speed`, so motorway speeds give a gentle wheel.
Controller sticks have a 15% dead zone and the same filter.

## Measured with the hatchback settings (tests/test_vehicle.c)

| Manoeuvre | Result |
|---|---|
| 0 to 50 km/h, full throttle | between 2 and 10 s (about 4 s) |
| Braking from 72 km/h | 24 m, under 5 s |
| Steady left at 36 km/h for 2 s | yaw > 0.3 rad, stays level |
| Dry versus 0.4 friction surface | wet stops at least 2 m/s faster still |
| Handbrake with steering at 54 km/h | rear wheels skid |

## Known limits

Ackermann geometry, load-sensitive tyre curves, wheel inertia and ABS
are not modelled. The chassis only collides at its corners, so a very
low kerb between the wheels passes under it. Wheels do not collide with
walls sideways; the chassis box does.
