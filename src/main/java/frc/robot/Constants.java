// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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

  public static class ShooterConstants {
    // Indexer motor CAN ID
    public static final int kIndexerMotorId = 5;
    public static final int kIndexerCurrentLimit = 20; // Amps
    
    // Turret motor CAN IDs
    public static final int kTurretMotorId = 6;
    public static final int kHoodMotorId = 7;
    public static final int kFlywheelMotorId = 8;
    
    // Current limits
    public static final int kTurretCurrentLimit = 30; // Amps
    public static final int kHoodCurrentLimit = 20; // Amps
    public static final int kFlywheelCurrentLimit = 40; // Amps
    
    // Gear ratios / conversion factors
    public static final double kTurretDegreesPerRotation = 360.0 / 50.0; // 50:1 gear ratio
    public static final double kHoodDegreesPerRotation = 360.0 / 25.0;   // 25:1 gear ratio
    
    // Turret angle limits (degrees)
    public static final double kTurretMinAngle = -180.0;
    public static final double kTurretMaxAngle = 180.0;
    
    // Hood angle limits (degrees)
    public static final double kHoodMinAngle = 0.0;
    public static final double kHoodMaxAngle = 60.0;
    
    // PID gains for turret position control
    public static final double kTurretP = 0.1;
    public static final double kTurretI = 0.0;
    public static final double kTurretD = 0.01;

    // PID gains for flywheel velocity control
    public static final double kFlywheelP = 0.075;
    public static final double kFlywheelI = 0.0;
    public static final double kFlywheelD = 0.0;
    public static final double kFlywheelFF = 0.000175; // Feedforward (V per RPM)
    
    // PID gains for hood position control
    public static final double kHoodP = 0.15;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.01;

    // Hood servo PWM channel
    public static final int kHoodServoChannel = 0;

    // Field dimensions (2025 Reefscape field)
    public static final double FIELD_LENGTH = 16.54; // meters
    public static final double FIELD_WIDTH = 8.21;   // meters

    // Limit RPM for shooting towards Driver Station
    public static final double kShootBackRPM = 3000.0;
    
    // Tower (goal) positions - approximate center of each alliance's tower
    // Blue tower is on the blue alliance side (x closer to 0)
    public static final Translation2d BLUE_HUB_POSITION = new Translation2d(4.5, FIELD_WIDTH / 2);
    // Red tower is on the red alliance side (x closer to field length)
    public static final Translation2d RED_HUB_POSITION = new Translation2d(FIELD_LENGTH - 4.5, FIELD_WIDTH / 2);
    
    // Tower height for calculating hood angle
    public static final double HUB_HEIGHT = 2.0; // meters - adjust based on actual goal height
    
    // Shooter height from ground
    public static final double SHOOTER_HEIGHT = 0.5; // meters - adjust based on robot
    
    // Driver station positions (for shooting back)
    public static final Translation2d BLUE_DRIVER_STATION = new Translation2d(0.5, FIELD_WIDTH / 2);
    public static final Translation2d RED_DRIVER_STATION = new Translation2d(FIELD_LENGTH - 0.5, FIELD_WIDTH / 2);
    
    // Hood angle lookup based on distance (distance in meters, angle in degrees)
    // These are placeholder values - tune based on testing
    public static final double[][] HOOD_ANGLE_TABLE = {
        // {distance, hood angle}
        {1.0, 60.0},
        {2.0, 50.0},
        {3.0, 40.0},
        {4.0, 35.0},
        {5.0, 30.0},
        {6.0, 25.0},
        {7.0, 22.0},
        {8.0, 20.0},
    };
    
    // Flywheel RPM lookup based on distance (replaces voltage table)
    public static final double[][] FLYWHEEL_RPM_TABLE = {
        // {distance in meters, target RPM}
        {1.0, 2000.0},
        {2.0, 2500.0},
        {3.0, 3000.0},
        {4.0, 3500.0},
        {5.0, 4000.0},
        {6.0, 4500.0},
        {7.0, 4800.0},
        {8.0, 5000.0},
    };
    
    /**
     * Interpolate a value from a lookup table
     */
    public static double interpolate(double[][] table, double input) {
        // Clamp to table bounds
        if (input <= table[0][0]) {
            return table[0][1];
        }
        if (input >= table[table.length - 1][0]) {
            return table[table.length - 1][1];
        }
        
        // Find surrounding entries and interpolate
        for (int i = 0; i < table.length - 1; i++) {
            if (input >= table[i][0] && input <= table[i + 1][0]) {
                double t = (input - table[i][0]) / (table[i + 1][0] - table[i][0]);
                return table[i][1] + t * (table[i + 1][1] - table[i][1]);
            }
        }
        
        return table[0][1]; // Fallback
    }
    
    public static double getHoodAngleForDistance(double distanceMeters) {
        return interpolate(HOOD_ANGLE_TABLE, distanceMeters);
    }
    
    public static double getFlywheelRPMForDistance(double distanceMeters) {
        return interpolate(FLYWHEEL_RPM_TABLE, distanceMeters);
    }
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
    
    // Control gains
    public static final double kIntakeForwardP = 2.0; // P gain for intake-forward driving

    // Robot dimensions (in meters)
    public static final double kTrackWidth = 0.6; // Distance between left and right wheels
    public static final double kWheelBase = 0.6;  // Distance between front and back wheels
    
    // Speed estimates for odometry (adjust based on testing)
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
  }
}
