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
