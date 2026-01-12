# Aiming Calculator Physics Documentation

## Overview
The AimingCalculator implements a projectile motion solver for a dual-turret shooting system. It accounts for robot motion, gravity, and mechanical constraints to calculate shooting parameters.

## Physical Constants

| Constant | Value | Description |
|----------|-------|-------------|
| Gravity (g) | 9.81 m/s² | Gravity |
| Flywheel Radius | 0.0381 m (1.5") | Radius of the shooting wheel |
| Max Flywheel RPM | 5600 | Maximum rotational speed |
| Launch Efficiency | 0.80 (80%) | Energy transfer efficiency from flywheel to projectile |
| Target Height | 1.8288 m (72") | Height of the scoring target |

## Turret Geometry

### Mounting Positions (from robot center)
- **Left Turret**: (0.2032m forward, 0.3556m left, 0.3556m up)
- **Right Turret**: (0.2032m forward, -0.3556m right, 0.3556m up)

### Field of View Constraints
- **Left Turret**: Can aim anywhere except 75° to 105° (cannot aim directly right)
- **Right Turret**: Can aim anywhere except -105° to -75° (cannot aim directly left)

## Core Physics Equations

### 1. Projectile Motion
The fundamental projectile motion equation in 2D:

```
y = x·tan(θ) - (g·x²)/(2·v₀²·cos²(θ))
```

Where:
- `y` = vertical displacement
- `x` = horizontal distance
- `θ` = launch angle (hood angle)
- `v₀` = initial velocity
- `g` = gravity

### 2. Velocity Calculation
Rearranging for initial velocity:

```
v₀² = (g·x²)/(2·cos²(θ)·(x·tan(θ) - Δh))
```

Where `Δh` is the height difference between target and shooter.

### 3. Flight Time
Time for projectile to reach target:

```
t = x/(v₀·cos(θ))
```

### 4. Flywheel Velocity Conversion
Converting between linear velocity and rotational speed:

```
RPM = (v·60)/(2π·r)
v = (RPM·2π·r)/60
```

## Motion Compensation

### Robot Velocity Component
The calculator accounts for robot motion by projecting velocity onto the shooting direction:

```
v_robot_component = v_x·cos(φ) + v_y·sin(φ)
```

Where `φ` is the angle from turret to target.

### Adjusted Launch Velocity
```
v_launch = v_required - v_robot_component·cos(θ)
```

This compensates for the robot's motion contribution to the projectile's initial velocity.

## Lead Compensation Algorithm

### 1. Future Position Prediction
Predicts where the robot will be when the projectile is launched:

```
x_future = x_current + v_x·t_flight
y_future = y_current + v_y·t_flight
θ_future = θ_current + ω·t_flight
```

### 2. Turret Position Transformation
Applies rotation transformation to find future turret position:

```
turret_field_pos = robot_pos + rotate(turret_offset, robot_heading)
```

### 3. Aiming Angle Calculation
Calculates the required turret angle in robot-relative coordinates:

```
field_angle = atan2(target_y - turret_y, target_x - turret_x)
robot_relative_angle = field_angle - robot_heading
```

## Optimization Strategy

### Trajectory Search
The algorithm iterates through hood angles (15° to 45°) to find the optimal trajectory:

1. For each hood angle:
   - Calculate required velocity using projectile equation
   - Adjust for robot motion
   - Check if achievable with max flywheel speed
   - Calculate flight time
   
2. Select trajectory with minimum launch velocity (most efficient)

### Efficiency Considerations
- **Launch Efficiency (80%)**: Accounts for energy loss in the shooting mechanism
- **Minimum Velocity Preference**: Reduces wear and improves accuracy
- **Fallback Strategy**: Uses distance-based interpolation if no perfect solution exists

## Coordinate Systems

[Coordinates](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)

### 1. Field Coordinate System
- Origin at field corner
- X-axis along field length
- Y-axis along field width
- Z-axis vertical (up positive)

### 2. Robot Coordinate System
- Origin at robot center
- X-axis forward
- Y-axis left
- Z-axis up

### 3. Turret Coordinate System
- Origin at turret pivot point
- Angle measured from robot forward direction
- Positive angles counter-clockwise

## Error Sources and Mitigation

### Physical Factors
1. **Air Resistance**: Not modeled, government progananda
2. **Magnus Effect**: Spin-induced lift not considered
3. **Projectile Deformation**: Assumes rigid body

### Mechanical Factors
1. **Flywheel Slip**: Compensated by efficiency factor
2. **Hood Angle Precision**: Limited by mechanical resolution
3. **Turret Backlash**: Not explicitly modeled

### Computational Factors
1. **Discrete Search**: 5° resolution for hood angle optimization
2. **Linear Approximation**: Robot motion assumed constant during flight
3. **Single Point Target**: Does not account for target size

## Performance Metrics

### Key Outputs
- **Turret Angle**: Robot-relative aiming direction
- **Hood Angle**: Launch angle for trajectory
- **Flywheel RPM**: Required wheel speed
- **Distance**: Calculated shot distance
- **Reachability**: Boolean indicating if target is within turret FOV

### Logged Parameters
The system logs all calculated parameters for debugging and tuning:
- Distance to target
- Hood angle selection
- Launch velocity
- Flight time
- Turret angle
- Flywheel RPM
- Target reachability

## Limitations and Assumptions

1. **Constant Acceleration**: Assumes uniform gravity field
2. **Point Mass**: Treats projectile as point mass
3. **Instantaneous Launch**: No acceleration phase modeled
4. **Perfect Tracking**: Assumes turret can track calculated angle perfectly
