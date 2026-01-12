package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class AimingCalculator {
    
    // Turret angle limits (in degrees, relative to turret's forward direction)
    // 0° = turret facing its default direction, positive = counterclockwise
    // Both turrets have ±120° range but cannot aim through the elevator (center of robot)
    
    // Left turret: default facing is robot's left side
    public static final double LEFT_TURRET_MIN_ANGLE = -120.0;
    public static final double LEFT_TURRET_MAX_ANGLE = 120.0;
    
    // Right turret: default facing is robot's right side
    public static final double RIGHT_TURRET_MIN_ANGLE = -120.0;
    public static final double RIGHT_TURRET_MAX_ANGLE = 120.0;
    
    /**
     * Calculate the turret angle to aim at a target position
     * @param robotPose Current robot pose
     * @param targetPosition Target position on the field
     * @param turretOnLeft Whether this turret is on the left side of the robot
     * @return Turret angle in degrees (0 = turret's forward, positive = counterclockwise)
     */
    public static double calculateTurretAngle(Pose2d robotPose, Translation2d targetPosition, boolean turretOnLeft) {
        // Vector from robot to target
        Translation2d robotToTarget = targetPosition.minus(robotPose.getTranslation());
        
        // Angle to target in field coordinates
        double fieldAngleToTarget = Math.atan2(robotToTarget.getY(), robotToTarget.getX());
        
        // Convert to robot-relative angle
        double robotAngle = robotPose.getRotation().getRadians();
        double robotRelativeAngle = fieldAngleToTarget - robotAngle;
        
        // Normalize to [-180, 180] degrees
        double angleDegrees = Math.toDegrees(robotRelativeAngle);
        angleDegrees = normalizeAngle(angleDegrees);
        
        // Log intermediate calculations
        String side = turretOnLeft ? "Left" : "Right";
        Logger.recordOutput("AimingCalculator/" + side + "/FieldAngleToTarget", Math.toDegrees(fieldAngleToTarget));
        Logger.recordOutput("AimingCalculator/" + side + "/RobotAngle", Math.toDegrees(robotAngle));
        Logger.recordOutput("AimingCalculator/" + side + "/RobotRelativeAngle", angleDegrees);
        
        // Adjust based on which side the turret is on
        // Left turret faces left (+90 from front), right turret faces right (-90 from front)
        if (turretOnLeft) {
            angleDegrees -= 90; // Turret's forward is robot's left
        } else {
            angleDegrees += 90; // Turret's forward is robot's right
        }
        
        double normalizedAngle = normalizeAngle(angleDegrees);
        Logger.recordOutput("AimingCalculator/" + side + "/TurretAngle", normalizedAngle);
        
        return normalizedAngle;
    }
    
    /**
     * Check if a turret can reach the target angle
     * @param turretAngle The calculated turret angle
     * @param turretOnLeft Whether this is the left turret
     * @return True if the turret can physically reach this angle
     */
    public static boolean canTurretReachAngle(double turretAngle, boolean turretOnLeft) {
        boolean canReach;
        if (turretOnLeft) {
            canReach = turretAngle >= LEFT_TURRET_MIN_ANGLE && turretAngle <= LEFT_TURRET_MAX_ANGLE;
        } else {
            canReach = turretAngle >= RIGHT_TURRET_MIN_ANGLE && turretAngle <= RIGHT_TURRET_MAX_ANGLE;
        }
        
        String side = turretOnLeft ? "Left" : "Right";
        Logger.recordOutput("AimingCalculator/" + side + "/CanReachAngle", canReach);
        
        return canReach;
    }
    
    /**
     * Clamp turret angle to valid range
     * @param turretAngle The desired turret angle
     * @param turretOnLeft Whether this is the left turret
     * @return The clamped angle within valid range
     */
    public static double clampTurretAngle(double turretAngle, boolean turretOnLeft) {
        double clampedAngle;
        if (turretOnLeft) {
            clampedAngle = Math.max(LEFT_TURRET_MIN_ANGLE, Math.min(LEFT_TURRET_MAX_ANGLE, turretAngle));
        } else {
            clampedAngle = Math.max(RIGHT_TURRET_MIN_ANGLE, Math.min(RIGHT_TURRET_MAX_ANGLE, turretAngle));
        }
        
        String side = turretOnLeft ? "Left" : "Right";
        Logger.recordOutput("AimingCalculator/" + side + "/ClampedAngle", clampedAngle);
        
        return clampedAngle;
    }
    
    /**
     * Calculate distance to a target
     */
    public static double calculateDistance(Pose2d robotPose, Translation2d targetPosition) {
        double distance = robotPose.getTranslation().getDistance(targetPosition);
        Logger.recordOutput("AimingCalculator/DistanceToTarget", distance);
        return distance;
    }
    
    /**
     * Get the target tower position based on alliance
     * @param targetOwnGoal If true, targets own alliance's goal; if false, targets opponent's
     */
    public static Translation2d getTargetTowerPosition() {
        var alliance = DriverStation.getAlliance();
        boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Blue;
        
        Translation2d position = isBlue ? ShooterConstants.BLUE_HUB_POSITION : ShooterConstants.RED_HUB_POSITION;
        
        Logger.recordOutput("AimingCalculator/Alliance", isBlue ? "Blue" : "Red");
        Logger.recordOutput("AimingCalculator/TargetTowerX", position.getX());
        Logger.recordOutput("AimingCalculator/TargetTowerY", position.getY());
        
        return position;
    }
    
    /**
     * Get the driver station position for shooting back
     */
    public static Translation2d getDriverStationPosition() {
        var alliance = DriverStation.getAlliance();
        boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Blue;
        
        Translation2d position = isBlue ? ShooterConstants.BLUE_DRIVER_STATION : ShooterConstants.RED_DRIVER_STATION;
        
        Logger.recordOutput("AimingCalculator/DriverStationX", position.getX());
        Logger.recordOutput("AimingCalculator/DriverStationY", position.getY());
        
        return position;
    }
    
    /**
     * Calculate hood angle based on distance
     */
    public static double calculateHoodAngle(double distanceMeters) {
        double angle = ShooterConstants.getHoodAngleForDistance(distanceMeters);
        Logger.recordOutput("AimingCalculator/HoodAngle", angle);
        return angle;
    }
    
    /**
     * Calculate flywheel RPM based on distance
     */
    public static double calculateFlywheelRPM(double distanceMeters) {
        double rpm = ShooterConstants.getFlywheelRPMForDistance(distanceMeters);
        Logger.recordOutput("AimingCalculator/FlywheelRPM", rpm);
        return rpm;
    }
    
    /**
     * Normalize angle to [-180, 180] degrees
     */
    private static double normalizeAngle(double angleDegrees) {
        while (angleDegrees > 180) angleDegrees -= 360;
        while (angleDegrees < -180) angleDegrees += 360;
        return angleDegrees;
    }
    
    /**
     * Calculate all aiming parameters for shooting at a target
     */
    public static AimingParameters calculateAimingParameters(
            Pose2d robotPose, 
            Translation2d targetPosition, 
            boolean turretOnLeft) {
        
        String side = turretOnLeft ? "Left" : "Right";
        
        // Log input parameters
        Logger.recordOutput("AimingCalculator/" + side + "/RobotPose", robotPose);
        Logger.recordOutput("AimingCalculator/" + side + "/TargetPosition", new Pose2d(targetPosition, new edu.wpi.first.math.geometry.Rotation2d()));
        
        double distance = calculateDistance(robotPose, targetPosition);
        double turretAngle = calculateTurretAngle(robotPose, targetPosition, turretOnLeft);
        boolean canReach = canTurretReachAngle(turretAngle, turretOnLeft);
        
        // Clamp to valid range if out of bounds
        double clampedAngle = clampTurretAngle(turretAngle, turretOnLeft);
        
        double hoodAngle = calculateHoodAngle(distance);
        double flywheelRPM = calculateFlywheelRPM(distance);
        
        // Log output parameters
        Logger.recordOutput("AimingCalculator/" + side + "/Parameters/TurretAngle", clampedAngle);
        Logger.recordOutput("AimingCalculator/" + side + "/Parameters/HoodAngle", hoodAngle);
        Logger.recordOutput("AimingCalculator/" + side + "/Parameters/FlywheelRPM", flywheelRPM);
        Logger.recordOutput("AimingCalculator/" + side + "/Parameters/Distance", distance);
        Logger.recordOutput("AimingCalculator/" + side + "/Parameters/CanReach", canReach);
        
        return new AimingParameters(clampedAngle, hoodAngle, flywheelRPM, distance, canReach);
    }
    
    /**
     * Container for all aiming parameters
     */
    public static record AimingParameters(
            double turretAngleDegrees,
            double hoodAngleDegrees,
            double flywheelRPM,
            double distanceMeters,
            boolean canReachTarget) {}
}
