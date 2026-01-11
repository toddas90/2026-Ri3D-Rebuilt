// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.DriveIO;
import frc.robot.subsystems.Drive.DriveIOSim;
import frc.robot.subsystems.Drive.DriveIOSparkMax;
import frc.robot.subsystems.Drive.GyroIO;
import frc.robot.subsystems.Drive.GyroIONavX;
import frc.robot.subsystems.Drive.GyroIOSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drive m_drive;

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  // Add operator controller on port 1
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // Deadband for joystick inputs
  private static final double DEADBAND = OperatorConstants.kControllerDeadband;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        m_drive = new Drive(new DriveIOSparkMax(), new GyroIONavX());
        break;
      case SIM:
        m_drive = new Drive(new DriveIOSim(), new GyroIOSim());
        break;
      default:
        m_drive = new Drive(new DriveIO() {}, new GyroIO() {});
    }

    // Configure the trigger bindings
    configureBindings();
    
    // Configure default commands
    configureDefaultCommands();
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // Driver Controller bindings
    // Reset gyro with Start button
    m_driverController.start().onTrue(Commands.runOnce(() -> m_drive.resetGyro()));
    
    // Operator Controller bindings
    // (Empty for now - add bindings here as needed)
  }

  /**
   * Configure default commands for subsystems
   */
  private void configureDefaultCommands() {
    // Set default command for drive to field-oriented control
    m_drive.setDefaultCommand(
        Commands.run(
            () -> {
              // Get joystick inputs
              double xSpeed = -m_driverController.getLeftY(); // Forward/backward (inverted)
              double ySpeed = -m_driverController.getLeftX(); // Left/right (inverted)
              double rotation = -m_driverController.getRightX(); // Rotation (inverted)
              
              // Apply deadband
              xSpeed = MathUtil.applyDeadband(xSpeed, DEADBAND);
              ySpeed = MathUtil.applyDeadband(ySpeed, DEADBAND);
              rotation = MathUtil.applyDeadband(rotation, DEADBAND);
              
              // Square inputs for finer control (while preserving sign)
              xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
              ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
              rotation = Math.copySign(rotation * rotation, rotation);
              
              // Drive field-oriented
              m_drive.driveFieldOriented(xSpeed, ySpeed, rotation);
            },
            m_drive
        )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return a simple auto that drives forward for 2 seconds
    return Commands.sequence(
        Commands.runOnce(() -> m_drive.resetGyro()),
        Commands.run(() -> m_drive.drive(0.3, 0, 0))
            .withTimeout(2.0)
            .finallyDo(() -> m_drive.stop())
    );
  }
}
