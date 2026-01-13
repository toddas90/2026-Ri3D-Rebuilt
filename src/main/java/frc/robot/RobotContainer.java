// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.SimpleShootCommand;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbIO;
import frc.robot.subsystems.Climb.ClimbIOSim;
import frc.robot.subsystems.Climb.ClimbIOSparkMax;
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
import frc.robot.subsystems.Shooter.Indexer;
import frc.robot.subsystems.Shooter.IndexerIO;
import frc.robot.subsystems.Shooter.IndexerIOSim;
import frc.robot.subsystems.Shooter.IndexerIOSparkMax;
import frc.robot.subsystems.Shooter.FixedShooter;
import frc.robot.subsystems.Shooter.FixedShooterAimingCalculator;
import frc.robot.subsystems.Shooter.FixedShooterIO;
import frc.robot.subsystems.Shooter.FixedShooterIOSparkMax;
import frc.robot.subsystems.Shooter.FixedShooterIOSim;
import edu.wpi.first.wpilibj.XboxController;
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

  private final Indexer m_indexer;
  private final Climb m_climb;
  private final FixedShooter m_fixedShooter;

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
        m_vision = new Vision(
            m_drive::addVisionMeasurement,
            new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
            new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1)
        );
        m_indexer = new Indexer(new IndexerIOSparkMax());
        m_climb = new Climb(new ClimbIOSparkMax());
        m_fixedShooter = new FixedShooter(new FixedShooterIOSparkMax());
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
        m_climb = new Climb(new ClimbIOSim());
        m_fixedShooter = new FixedShooter(new FixedShooterIOSim());
        break;

      default:
        m_drive = new Drive(new DriveIO() {}, new GyroIO() {});
        m_vision = new Vision(m_drive::addVisionMeasurement);
        m_indexer = new Indexer(new IndexerIO() {});
        m_climb = new Climb(new ClimbIO() {});
        m_fixedShooter = new FixedShooter(new FixedShooterIO() {});
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
    
    // ==================== LIFT CONTROLS ====================
    
    // D-Pad Up: Move lift to TOP position
    m_driverController.povUp().onTrue(
        Commands.runOnce(() -> m_climb.setLiftPosition(Climb.LiftPosition.EXTENDED), m_climb)
    );
    
    // D-Pad Right: Move lift to MIDDLE position for bar insertion
    m_driverController.povRight().onTrue(
        Commands.runOnce(() -> m_climb.setLiftPosition(Climb.LiftPosition.BAR_INSERT), m_climb)
    );
    
    // D-Pad Down: Move lift to BOTTOM position
    m_driverController.povDown().onTrue(
        Commands.runOnce(() -> m_climb.setLiftPositionSmart(Climb.LiftPosition.STOWED), m_climb)
    );
    
    // B Button: Flip robot (toggle between NORMAL and FLIPPED)
    m_driverController.b().onTrue(
        Commands.either(
            Commands.runOnce(() -> m_climb.setPivotPosition(Climb.PivotPosition.NORMAL), m_climb),
            Commands.runOnce(() -> m_climb.setPivotPosition(Climb.PivotPosition.FLIPPED), m_climb),
            () -> m_climb.getPivotAngle() > 90.0
        )
    );
    
    // Back button: Emergency stop for climb
    m_driverController.back().onTrue(
        Commands.runOnce(() -> {
            m_climb.stop();
            m_climb.setBrakeMode(true);
        }, m_climb)
    );

    // Left Bumper: Simple shoot without aiming
    m_driverController.leftBumper().whileTrue(
        new SimpleShootCommand(m_fixedShooter)
    );

    // Right Bumper: Auto-aim and spin up
    m_driverController.rightBumper().whileTrue(
        new AutoAimCommand(
            m_drive,
            m_fixedShooter,
            FixedShooterAimingCalculator::getTargetPosition
        )
    );

    // X button: Fire (with indexer)
    m_driverController.x().whileTrue(
        Commands.startEnd(
            () -> m_indexer.start(),
            () -> m_indexer.stop(),
            m_indexer
        )
    );
  }

  private void configureDefaultCommands() {
    // Set default command for drive to field-oriented control
    m_drive.setDefaultCommand(
        new FieldOrientedDriveCommand(
            m_drive,
            () -> -m_driverController.getLeftY(),  // Forward/backward (inverted)
            () -> m_driverController.getLeftX(),  // Left/right (inverted)
            () -> -m_driverController.getRightX(), // Rotation (inverted) ?????
            DEADBAND
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
