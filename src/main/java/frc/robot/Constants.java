// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kControllerDeadband = 0.05;
  }

  public static class DriveConstants {
    // Motor CAN IDs
    public static final int kFrontLeftMotorId = 1;
    public static final int kFrontRightMotorId = 2;
    public static final int kRearLeftMotorId = 3;
    public static final int kRearRightMotorId = 4;

    // Wheel offsets from robot center (in meters)
    public static final Translation2d kFrontLeftWheelOffset = new Translation2d(-0.37, 0.2);
    public static final Translation2d kFrontRightWheelOffset = new Translation2d(0.37, 0.2);
    public static final Translation2d kRearLeftWheelOffset = new Translation2d(0.37, -0.2);
    public static final Translation2d kRearRightWheelOffset = new Translation2d(-0.37, -0.2);
    
    // Motor configuration
    public static final int kCurrentLimit = 40; // Amps
    public static final double kMaxVoltage = 12.0; // Volts
    
    // Robot dimensions (in meters)
    public static final double kTrackWidth = 0.6; // Distance between left and right wheels
    public static final double kWheelBase = 0.6;  // Distance between front and back wheels
    
    // Speed limits
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    
    // Control gains
    public static final double kIntakeForwardP = 2.0; // P gain for intake-forward driving
  }

  public static class IntakeConstants {
    // Motor IDs
    public static final int kPivotMotorId = 9;  // TODO: Update with actual CAN ID
    public static final int kRollerMotorId = 10; // TODO: Update with actual CAN ID
    
    // Positions (degrees)
    public static final double kStowedPosition = 0.0;
    public static final double kDeployedPosition = 90.0;
    
    // Speeds
    public static final double kIntakeVoltage = 8.0;
    public static final double kEjectVoltage = -4.0;
    
    // Current limits
    public static final int kPivotCurrentLimit = 30;
    public static final int kRollerCurrentLimit = 20;
    public static final double kRollerStallCurrent = 15.0;
    
    // PID values (tune these)
    public static final double kPivotP = 0.1;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.0;
  }

  public static class ShooterConstants {
    // Motor CAN IDs
    public static final int kIndexerMotorId = 5;
    public static final int kTurretMotorId = 6;
    public static final int kHoodMotorId = 7;    // Note: Hood uses servo, not motor
    public static final int kFlywheelMotorId = 8;
    
    // Current limits
    public static final int kIndexerCurrentLimit = 20; // Amps
    public static final int kTurretCurrentLimit = 30; // Amps
    public static final int kHoodCurrentLimit = 20; // Amps
    public static final int kFlywheelCurrentLimit = 40; // Amps
    
    // Mechanical ratios
    public static final double kTurretDegreesPerRotation = 360.0 / 50.0; // 50:1 gear ratio
    public static final double kHoodDegreesPerRotation = 360.0 / 25.0;   // 25:1 gear ratio
    
    // Position limits (degrees)
    public static final double kTurretMinAngle = -180.0;
    public static final double kTurretMaxAngle = 180.0;
    public static final double kHoodMinAngle = 0.0;
    public static final double kHoodMaxAngle = 60.0;
    
    // Turret PID
    public static final double kTurretP = 0.1;
    public static final double kTurretI = 0.0;
    public static final double kTurretD = 0.01;

    // Flywheel PID
    public static final double kFlywheelP = 0.075;
    public static final double kFlywheelI = 0.0;
    public static final double kFlywheelD = 0.0;
    public static final double kFlywheelFF = 0.000175; // Feedforward (V per RPM)
    
    // Hood PID
    public static final double kHoodP = 0.15;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.01;

    // Hardware config
    public static final int kHoodServoChannel = 0; // PWM channel

    // ==================== AIMING PHYSICS CONSTANTS ====================
    
    // Physical Constants
    public static final double kGravity = 9.81; // m/s^2
    public static final double kFlywheelRadius = 1.5 * 0.0254; // 1.5" radius in meters
    public static final double kMaxFlywheelRPM = 5600.0;
    public static final double kLaunchEfficiency = 0.80; // Energy transfer efficiency
    
    // Hood angle limits for trajectory
    public static final double kMinHoodAngle = 45.0; // degrees - distance shot
    public static final double kMaxHoodAngle = 75.0; // degrees - upward shot
    
    // Turret mounting positions (from robot center)
    public static final Translation3d kRightTurretPosition = new Translation3d(
        8.0 * 0.0254,   // 0.2032m forward (X)
        -14.0 * 0.0254, // -0.3556m right (-Y)
        14.0 * 0.0254   // 0.3556m up (Z)
    );
    
    public static final Translation3d kLeftTurretPosition = new Translation3d(
        8.0 * 0.0254,  // 0.2032m forward (X)
        14.0 * 0.0254, // 0.3556m left (+Y)
        14.0 * 0.0254  // 0.3556m up (Z)
    );
    
    // Turret FOV limits (in degrees, 0° = robot forward, positive CCW)
    // Left turret: can aim anywhere except directly right (-90° ± 22.5°)
    public static final double kLeftTurretDeadzoneMin = -112.5;  // -90° - 22.5°
    public static final double kLeftTurretDeadzoneMax = -67.5;   // -90° + 22.5°
    
    // Right turret: can aim anywhere except directly left (90° ± 22.5°)
    public static final double kRightTurretDeadzoneMin = 67.5;   // 90° - 22.5°
    public static final double kRightTurretDeadzoneMax = 112.5;  // 90° + 22.5°
    
    // ==================== FIELD CONSTANTS ====================
    
    // Shooting parameters
    public static final double kShootBackRPM = 3000.0; // RPM limit for driver station shots
    public static final double SHOOTER_HEIGHT = 0.381; // 15 inches in meters (deprecated - use turret positions)
    
    // Field-specific constants
    public static final double FIELD_LENGTH = 16.54; // meters
    public static final double FIELD_WIDTH = 8.21;   // meters
    public static final double HUB_HEIGHT = 1.8288; // 72 inches in meters
    
    // Alliance-specific targets
    public static final Translation2d BLUE_HUB_POSITION = new Translation2d(4.5, FIELD_WIDTH / 2);
    public static final Translation2d RED_HUB_POSITION = new Translation2d(FIELD_LENGTH - 4.5, FIELD_WIDTH / 2);
    public static final Translation2d BLUE_DRIVER_STATION = new Translation2d(0.5, FIELD_WIDTH / 2);
    public static final Translation2d RED_DRIVER_STATION = new Translation2d(FIELD_LENGTH - 0.5, FIELD_WIDTH / 2);
  }

  public static class ClimbConstants {
    // Motor CAN IDs
    public static final int kLiftMotorId = 30;
    public static final int kPivotMotorId = 31;
    
    // Mechanical configuration
    public static final double kLiftGearRatio = 25.0; // 25:1 reduction
    public static final double kPivotGearRatio = 100.0; // 100:1 reduction
    public static final double kLiftDrumRadius = 0.0254; // 1" radius in meters
    
    // Conversion factors (fixed calculation)
    public static final double kLiftMetersPerRotation = 2 * Math.PI * kLiftDrumRadius * kLiftGearRatio;
    public static final double kPivotDegreesPerRotation = 360.0 / kPivotGearRatio;
    
    // Position limits
    public static final double kLiftMinHeight = 0.0; // meters
    public static final double kLiftMaxHeight = 0.5; // meters
    public static final double kLiftBarInsertHeight = 0.4; // meters
    public static final double kPivotMinAngle = 0.0; // degrees
    public static final double kPivotMaxAngle = 180.0; // degrees (full flip)
    
    // Lift PID
    public static final double kLiftP = 5.0;
    public static final double kLiftI = 0.0;
    public static final double kLiftD = 0.1;
    public static final double kLiftFF = 0.0;
    
    // Pivot PID
    public static final double kPivotP = 0.02;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.001;
    public static final double kPivotFF = 0.0;
    
    // Current limits
    public static final int kLiftCurrentLimit = 40; // amps
    public static final int kPivotCurrentLimit = 40; // amps
    
    // Sensors
    public static final int kLiftBottomLimitPort = 0; // Digital IO
    public static final int kLiftTopLimitPort = 1;    // Digital IO
    
    // Control tolerances
    public static final double kLiftPositionTolerance = 0.02; // meters
    public static final double kPivotPositionTolerance = 2.0; // degrees
    
    // Safety limits
    public static final double kMaxMotorTemp = 70.0; // Celsius
  }
}
