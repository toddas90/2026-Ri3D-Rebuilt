// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AimAtTargetCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.commands.ManualTurretAimCommand;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.DriveIO;
import frc.robot.subsystems.Drive.DriveIOSim;
import frc.robot.subsystems.Drive.DriveIOSparkMax;
import frc.robot.subsystems.Drive.GyroIO;
import frc.robot.subsystems.Drive.GyroIOSim;
import frc.robot.subsystems.Drive.GyroIONavX;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIOPhotonVision;
import frc.robot.subsystems.Vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.subsystems.Shooter.Turret;
import frc.robot.subsystems.Shooter.TurretIO;
import frc.robot.subsystems.Shooter.TurretIOSim;
import frc.robot.subsystems.Shooter.Indexer;
import frc.robot.subsystems.Shooter.IndexerIO;
import frc.robot.subsystems.Shooter.IndexerIOSim;
import frc.robot.subsystems.Shooter.IndexerIOSparkMax;
import frc.robot.subsystems.Shooter.TurretIOSparkMax;
import frc.robot.subsystems.Shooter.AimingCalculator;
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
  private final Vision m_vision;

  private final Turret m_leftTurret;
  private final Turret m_rightTurret;
  private final Indexer m_indexer;

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  // Add operator controller on port 1
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // Deadband for joystick inputs
  private static final double DEADBAND = OperatorConstants.kControllerDeadband;
  
  // Max RPM for shooting back to driver station
  private static final double MAX_SHOOT_BACK_RPM = ShooterConstants.kShootBackRPM;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        m_drive = new Drive(new DriveIOSparkMax(), new GyroIONavX());
        m_vision = new Vision(
            m_drive::addVisionMeasurement,
            new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
            new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1)
        );
        m_indexer = new Indexer(new IndexerIOSparkMax());
        m_leftTurret = new Turret(new TurretIOSparkMax(), "LeftTurret");
        m_rightTurret = new Turret(new TurretIOSparkMax(), "RightTurret");
        break;

      case SIM:
        m_drive = new Drive(new DriveIOSim(), new GyroIOSim());
        m_vision = new Vision(
            m_drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(
                VisionConstants.camera0Name, 
                VisionConstants.robotToCamera0, 
                m_drive::getPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.camera1Name, 
                VisionConstants.robotToCamera1, 
                m_drive::getPose)
        );
        m_indexer = new Indexer(new IndexerIOSim());
        m_leftTurret = new Turret(new TurretIOSim(), "LeftTurret");
        m_rightTurret = new Turret(new TurretIOSim(), "RightTurret");
        break;

      default:
        m_drive = new Drive(new DriveIO() {}, new GyroIO() {});
        m_vision = new Vision(m_drive::addVisionMeasurement);
        m_indexer = new Indexer(new IndexerIO() {});
        m_leftTurret = new Turret(new TurretIO() {}, "LeftTurret");
        m_rightTurret = new Turret(new TurretIO() {}, "RightTurret");
    }

    // Configure the trigger bindings
    configureBindings();
    
    // Configure default commands
    configureDefaultCommands();
  }

  private void configureBindings() {
    // Driver Controller bindings
    // Reset gyro with Start button
    m_driverController.start().onTrue(Commands.runOnce(() -> m_drive.resetGyro()));

    // ==================== SHOOTING CONTROLS ====================
    
    // Left Trigger: Aim towards own driver station (shoot back)
    m_driverController.rightTrigger(0.5).whileTrue( // operator
        new AimAtTargetCommand(
            m_leftTurret,
            m_rightTurret,
            m_drive::getPose,
            AimingCalculator::getDriverStationPosition,
            m_drive::getChassisSpeeds,
            "DriverStation",
            MAX_SHOOT_BACK_RPM // Limit RPM for gentler shot
        )
    );
    
    // Right Trigger: Aim at hub
    m_driverController.leftTrigger(0.5).whileTrue( // operator
        new AimAtTargetCommand(
            m_leftTurret,
            m_rightTurret,
            m_drive::getPose,
            AimingCalculator::getTargetTowerPosition,
            m_drive::getChassisSpeeds,
            "Tower"
        )
    );
    
    // ==================== OPERATOR CONTROLLER BINDINGS ====================

    // Manual Turret Aiming - Left Bumper held + Left Stick controls left turret
    m_driverController.leftBumper().whileTrue( // operator
        new ManualTurretAimCommand(
            m_leftTurret,
            () -> m_operatorController.getLeftX(),
            () -> -m_operatorController.getLeftY(), // Inverted Y
            0.5 // Deadband
        )
    );

    // Manual Turret Aiming - Right Bumper held + Right Stick controls right turret
    m_driverController.rightBumper().whileTrue( // operator
        new ManualTurretAimCommand(
            m_rightTurret,
            () -> m_operatorController.getRightX(),
            () -> -m_operatorController.getRightY(), // Inverted Y
            0.5 // Deadband
        )
    );
    
    // A Button: Manual shoot (spin up flywheels and run indexer)
    m_driverController.a().whileTrue( // operator
        new ManualShootCommand(m_leftTurret, m_rightTurret, m_indexer)
    );
  }

  private void configureDefaultCommands() {
    // Set default command for drive to field-oriented control
    m_drive.setDefaultCommand(
        new FieldOrientedDriveCommand(
            m_drive,
            () -> -m_driverController.getLeftY(),  // Forward/backward (inverted)
            () -> -m_driverController.getLeftX(),  // Left/right (inverted)
            () -> -m_driverController.getRightX(), // Rotation (inverted) ?????
            DEADBAND
        )
    );
    
    // Default command for turrets: continuously aim at hub WITHOUT spinning flywheels
    Command aimAtHubCommand = new AimAtTargetCommand(
        m_leftTurret,
        m_rightTurret,
        m_drive::getPose,
        AimingCalculator::getTargetTowerPosition,
        m_drive::getChassisSpeeds,  // Add velocity supplier
        "Tower",
        0.0  // maxRPM = 0 means no flywheel spin-up
    );
    m_leftTurret.setDefaultCommand(aimAtHubCommand);
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
