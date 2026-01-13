package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Shooter.FixedShooter;
import frc.robot.subsystems.Shooter.FixedShooterAimingCalculator;
import frc.robot.subsystems.Shooter.FixedShooterAimingCalculator.FixedAimingParameters;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

/**
 * Command that rotates the robot to aim at target and spins up flywheels
 */
public class AutoAimCommand extends Command {
    private final Drive drive;
    private final FixedShooter shooter;
    private final Supplier<Translation2d> targetSupplier;
    private final PIDController rotationController;
    
    private static final double ROTATION_kP = 0.1;
    private static final double ROTATION_kI = 0.0;
    private static final double ROTATION_kD = 0.01;
    private static final double MAX_ROTATION_SPEED = 3.0; // rad/s
    
    public AutoAimCommand(Drive drive, FixedShooter shooter, Supplier<Translation2d> targetSupplier) {
        this.drive = drive;
        this.shooter = shooter;
        this.targetSupplier = targetSupplier;
        
        this.rotationController = new PIDController(ROTATION_kP, ROTATION_kI, ROTATION_kD);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Math.toRadians(2.0)); // 2 degree tolerance
        
        addRequirements(drive, shooter);
    }
    
    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();
        ChassisSpeeds robotVelocity = drive.getVelocity();
        Translation2d targetPosition = targetSupplier.get();
        
        // Calculate aiming parameters
        FixedAimingParameters params = FixedShooterAimingCalculator.calculateAiming(
            robotPose, robotVelocity, targetPosition);
        
        // Calculate rotation speed to align with target
        double currentHeading = robotPose.getRotation().getRadians();
        double targetHeading = params.targetHeading().getRadians();
        double rotationSpeed = rotationController.calculate(currentHeading, targetHeading);
        
        // Clamp rotation speed
        rotationSpeed = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotationSpeed));
        
        // Drive with rotation only (maintain any existing translation from driver)
        drive.driveFieldOriented(
            drive.getVelocity().vxMetersPerSecond,
            drive.getVelocity().vyMetersPerSecond,
            rotationSpeed
        );
        
        // Spin up flywheels to calculated RPM
        shooter.setFlywheelVelocity(params.flywheelRPM());
        
        // Log status
        Logger.recordOutput("AutoAim/AtTargetHeading", rotationController.atSetpoint());
        Logger.recordOutput("AutoAim/HeadingError", Math.toDegrees(targetHeading - currentHeading));
        Logger.recordOutput("AutoAim/TargetRPM", params.flywheelRPM());
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop rotating but keep flywheels spinning
        drive.driveFieldOriented(0, 0, 0);
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
    
    /**
     * Check if robot is aimed at target
     */
    public boolean isAimed() {
        return rotationController.atSetpoint();
    }
}