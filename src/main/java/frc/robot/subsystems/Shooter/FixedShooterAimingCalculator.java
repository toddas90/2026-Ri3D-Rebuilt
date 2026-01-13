package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class FixedShooterAimingCalculator {
    
    // Fixed hood angle for all shots
    private static final double FIXED_HOOD_ANGLE = ShooterConstants.kHoodAngle; // degrees
    
    /**
     * Calculate robot rotation and flywheel speed for fixed shooter
     */
    public static FixedAimingParameters calculateAiming(
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Translation2d targetPosition) {
        
        // Calculate distance from robot center to target
        double distance = robotPose.getTranslation().getDistance(targetPosition);
        
        // Calculate required robot heading to face target
        Translation2d toTarget = targetPosition.minus(robotPose.getTranslation());
        double targetHeading = Math.atan2(toTarget.getY(), toTarget.getX());
        
        // Calculate RPM based on distance (simplified physics)
        double flywheelRPM = calculateRPMForDistance(distance, robotPose, robotVelocity, targetPosition);
        
        // Log calculations
        Logger.recordOutput("FixedShooterAiming/Distance", distance);
        Logger.recordOutput("FixedShooterAiming/TargetHeading", Math.toDegrees(targetHeading));
        Logger.recordOutput("FixedShooterAiming/CurrentHeading", robotPose.getRotation().getDegrees());
        Logger.recordOutput("FixedShooterAiming/FlywheelRPM", flywheelRPM);
        Logger.recordOutput("FixedShooterAiming/HoodAngle", FIXED_HOOD_ANGLE);
        
        return new FixedAimingParameters(
            new Rotation2d(targetHeading),
            flywheelRPM,
            distance,
            FIXED_HOOD_ANGLE
        );
    }
    
    /**
     * Calculate required flywheel RPM for distance with fixed hood angle
     */
    private static double calculateRPMForDistance(
            double distance,
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Translation2d targetPosition) {
        
        // Fixed angle shooting physics
        double angleRad = Math.toRadians(FIXED_HOOD_ANGLE);
        double cosAngle = Math.cos(angleRad);
        double tanAngle = Math.tan(angleRad);
        
        // Height difference (assuming robot center height)
        double shooterHeight = ShooterConstants.SHOOTER_HEIGHT; // From turret mounting position
        double heightDiff = ShooterConstants.HUB_HEIGHT - shooterHeight;
        
        // Calculate required launch velocity using projectile motion
        double denominator = 2 * cosAngle * cosAngle * (distance * tanAngle - heightDiff);
        if (denominator <= 0) {
            // Can't reach target with this angle, use max RPM
            return ShooterConstants.kMaxFlywheelRPM;
        }
        
        double velocitySquared = (ShooterConstants.kGravity * distance * distance) / denominator;
        double requiredVelocity = Math.sqrt(velocitySquared);
        
        // Account for robot motion in shooting direction
        Translation2d toTarget = targetPosition.minus(robotPose.getTranslation());
        double shootAngle = Math.atan2(toTarget.getY(), toTarget.getX());
        double robotSpeed = robotVelocity.vxMetersPerSecond * Math.cos(shootAngle) +
                           robotVelocity.vyMetersPerSecond * Math.sin(shootAngle);
        
        double launchVelocity = requiredVelocity - robotSpeed * Math.cos(angleRad);
        
        // Convert to RPM
        double rpm = velocityToRPM(launchVelocity / ShooterConstants.kLaunchEfficiency);
        
        // Clamp to achievable range
        return Math.min(Math.max(rpm, 0), ShooterConstants.kMaxFlywheelRPM);
    }
    
    /**
     * Convert velocity to flywheel RPM
     */
    private static double velocityToRPM(double velocity) {
        double angularVelocity = velocity / ShooterConstants.kFlywheelRadius;
        double flywheelRPM = angularVelocity * 60.0 / (2 * Math.PI);
        return flywheelRPM * ShooterConstants.kFlywheelGearRatio;
    }
    
    /**
     * Get target position based on alliance
     */
    public static Translation2d getTargetPosition() {
        var alliance = DriverStation.getAlliance();
        boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Blue;
        return isBlue ? ShooterConstants.BLUE_HUB_POSITION : ShooterConstants.RED_HUB_POSITION;
    }
    
    /**
     * Result of fixed shooter aiming calculation
     */
    public static record FixedAimingParameters(
        Rotation2d targetHeading,
        double flywheelRPM,
        double distanceMeters,
        double hoodAngleDegrees
    ) {}
}