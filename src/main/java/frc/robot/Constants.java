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
    
    // PID gains for hood position control
    public static final double kHoodP = 0.15;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.01;
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
