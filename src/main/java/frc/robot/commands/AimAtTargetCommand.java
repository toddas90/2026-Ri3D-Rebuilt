package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
            String modeName) {
        this(leftTurret, rightTurret, poseSupplier, targetSupplier, modeName, Double.MAX_VALUE);
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
            String modeName,
            double maxRPM) {
        this.leftTurret = leftTurret;
        this.rightTurret = rightTurret;
        this.poseSupplier = poseSupplier;
        this.targetSupplier = targetSupplier;
        this.modeName = modeName;
        this.maxRPM = maxRPM;

        addRequirements(leftTurret, rightTurret);
    }

    @Override
    public void execute() {
        Translation2d targetPosition = targetSupplier.get();
        Pose2d currentPose = poseSupplier.get();

        // Calculate aiming parameters for left turret
        AimingParameters leftParams = AimingCalculator.calculateAimingParameters(
                currentPose,
                targetPosition,
                true // left turret
        );

        // Calculate aiming parameters for right turret
        AimingParameters rightParams = AimingCalculator.calculateAimingParameters(
                currentPose,
                targetPosition,
                false // right turret
        );

        // Apply aiming to turrets (only spin up flywheel if turret can reach target)
        applyAiming(leftTurret, leftParams, "Left");
        applyAiming(rightTurret, rightParams, "Right");

        // Log aiming data
        logAimingData(targetPosition, leftParams, rightParams);
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

    private void logAimingData(Translation2d targetPosition, AimingParameters leftParams, AimingParameters rightParams) {
        Logger.recordOutput("Shooter/Aiming/Mode", modeName);
        Logger.recordOutput("Shooter/Aiming/TargetX", targetPosition.getX());
        Logger.recordOutput("Shooter/Aiming/TargetY", targetPosition.getY());
        Logger.recordOutput("Shooter/Aiming/LeftDistance", leftParams.distanceMeters());
        Logger.recordOutput("Shooter/Aiming/LeftTurretAngle", leftParams.turretAngleDegrees());
        Logger.recordOutput("Shooter/Aiming/LeftHoodAngle", leftParams.hoodAngleDegrees());
        Logger.recordOutput("Shooter/Aiming/LeftFlywheelRPM", leftParams.flywheelRPM());
        Logger.recordOutput("Shooter/Aiming/LeftCanReach", leftParams.canReachTarget());
        Logger.recordOutput("Shooter/Aiming/RightDistance", rightParams.distanceMeters());
        Logger.recordOutput("Shooter/Aiming/RightTurretAngle", rightParams.turretAngleDegrees());
        Logger.recordOutput("Shooter/Aiming/RightHoodAngle", rightParams.hoodAngleDegrees());
        Logger.recordOutput("Shooter/Aiming/RightFlywheelRPM", rightParams.flywheelRPM());
        Logger.recordOutput("Shooter/Aiming/RightCanReach", rightParams.canReachTarget());
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
