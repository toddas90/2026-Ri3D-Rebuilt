package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.AimingCalculator;
import frc.robot.subsystems.Shooter.AimingCalculator.AimingParameters;
import frc.robot.subsystems.Shooter.Turret;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

/**
 * Command that continuously aims both turrets at a target position.
 */
public class AimAtTargetCommand extends Command {
    private final Turret leftTurret;
    private final Turret rightTurret;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Translation2d> targetSupplier;
    private final String modeName;
    private final double maxRPM;
        private Supplier<ChassisSpeeds> velocitySupplier;
    
        /**
         * Creates a command that aims both turrets at a target.
         * 
         * @param leftTurret The left turret subsystem
         * @param rightTurret The right turret subsystem
         * @param poseSupplier Supplier for the current robot pose
         * @param targetSupplier Supplier for the target position
         * @param modeName Name for logging purposes
         */
        public AimAtTargetCommand(
                Turret leftTurret,
                Turret rightTurret,
                Supplier<Pose2d> poseSupplier,
                Supplier<Translation2d> targetSupplier,
                Supplier<ChassisSpeeds> velocitySupplier,
                String modeName) {
            this(leftTurret, rightTurret, poseSupplier, targetSupplier, velocitySupplier, modeName, Double.MAX_VALUE);
        }
    
        /**
         * Creates a command that aims both turrets at a target with a max RPM cap.
         * 
         * @param leftTurret The left turret subsystem
         * @param rightTurret The right turret subsystem
         * @param poseSupplier Supplier for the current robot pose
         * @param targetSupplier Supplier for the target position
         * @param modeName Name for logging purposes
         * @param maxRPM Maximum flywheel RPM (for gentler shots)
         */
        public AimAtTargetCommand(
                Turret leftTurret,
                Turret rightTurret,
                Supplier<Pose2d> poseSupplier,
                Supplier<Translation2d> targetSupplier,
                Supplier<ChassisSpeeds> velocitySupplier,
                String modeName,
                double maxRPM) {
            this.leftTurret = leftTurret;
            this.rightTurret = rightTurret;
            this.poseSupplier = poseSupplier;
            this.targetSupplier = targetSupplier;
            this.velocitySupplier = velocitySupplier;
        this.modeName = modeName;
        this.maxRPM = maxRPM;

        addRequirements(leftTurret, rightTurret);
    }

    @Override
    public void execute() {
        Pose2d robotPose = poseSupplier.get();
        ChassisSpeeds robotVelocity = velocitySupplier.get();
        Translation2d targetPosition = targetSupplier.get();
        
        // Calculate aiming parameters for each turret with velocity compensation
        var leftParams = AimingCalculator.calculateAiming(
                robotPose, robotVelocity, targetPosition, true);
        var rightParams = AimingCalculator.calculateAiming(
                robotPose, robotVelocity, targetPosition, false);
        
        // Apply aiming to each turret
        applyAiming(leftTurret, leftParams, "Left");
        applyAiming(rightTurret, rightParams, "Right");
        
        // Log targeting data
        Logger.recordOutput("AimAtTarget/" + modeName + "/RobotPose", robotPose);
        Logger.recordOutput("AimAtTarget/" + modeName + "/TargetPosition", targetPosition);
        Logger.recordOutput("AimAtTarget/" + modeName + "/RobotVelocityX", robotVelocity.vxMetersPerSecond);
        Logger.recordOutput("AimAtTarget/" + modeName + "/RobotVelocityY", robotVelocity.vyMetersPerSecond);
    }

    private void applyAiming(Turret turret, AimingParameters params, String side) {
        if (params.canReachTarget()) {
            double rpm = Math.min(params.flywheelRPM(), maxRPM);
            turret.prepareShotVelocity(
                    params.turretAngleDegrees(),
                    params.hoodAngleDegrees(),
                    rpm);
        } else {
            // Can't reach - move to limit and don't spin flywheel
            turret.aim(params.turretAngleDegrees(), params.hoodAngleDegrees());
            turret.stopFlywheel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop flywheels when aiming is released
        leftTurret.stopFlywheel();
        rightTurret.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}

