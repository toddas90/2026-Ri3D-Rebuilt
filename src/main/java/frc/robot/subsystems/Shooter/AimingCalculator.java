package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ShooterConstants;

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
        
        // Adjust based on which side the turret is on
        // Left turret faces left (+90 from front), right turret faces right (-90 from front)
        if (turretOnLeft) {
            angleDegrees -= 90; // Turret's forward is robot's left
        } else {
            angleDegrees += 90; // Turret's forward is robot's right
        }
        
        return normalizeAngle(angleDegrees);
    }
    
    /**
     * Check if a turret can reach the target angle
     * @param turretAngle The calculated turret angle
     * @param turretOnLeft Whether this is the left turret
     * @return True if the turret can physically reach this angle
     */
    public static boolean canTurretReachAngle(double turretAngle, boolean turretOnLeft) {
        if (turretOnLeft) {
            return turretAngle >= LEFT_TURRET_MIN_ANGLE && turretAngle <= LEFT_TURRET_MAX_ANGLE;
        } else {
            return turretAngle >= RIGHT_TURRET_MIN_ANGLE && turretAngle <= RIGHT_TURRET_MAX_ANGLE;
        }
    }
    
    /**
     * Clamp turret angle to valid range
     * @param turretAngle The desired turret angle
     * @param turretOnLeft Whether this is the left turret
     * @return The clamped angle within valid range
     */
    public static double clampTurretAngle(double turretAngle, boolean turretOnLeft) {
        if (turretOnLeft) {
            return Math.max(LEFT_TURRET_MIN_ANGLE, Math.min(LEFT_TURRET_MAX_ANGLE, turretAngle));
        } else {
            return Math.max(RIGHT_TURRET_MIN_ANGLE, Math.min(RIGHT_TURRET_MAX_ANGLE, turretAngle));
        }
    }
    
    /**
     * Calculate distance to a target
     */
    public static double calculateDistance(Pose2d robotPose, Translation2d targetPosition) {
        return robotPose.getTranslation().getDistance(targetPosition);
    }
    
    /**
     * Get the target tower position based on alliance
     * @param targetOwnGoal If true, targets own alliance's goal; if false, targets opponent's
     */
    public static Translation2d getTargetTowerPosition() {
        var alliance = DriverStation.getAlliance();
        boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Blue;
        
        return isBlue ? ShooterConstants.BLUE_HUB_POSITION : ShooterConstants.RED_HUB_POSITION;
    }
    
    /**
     * Get the driver station position for shooting back
     */
    public static Translation2d getDriverStationPosition() {
        var alliance = DriverStation.getAlliance();
        boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Blue;
        
        return isBlue ? ShooterConstants.BLUE_DRIVER_STATION : ShooterConstants.RED_DRIVER_STATION;
    }
    
    /**
     * Calculate hood angle based on distance
     */
    public static double calculateHoodAngle(double distanceMeters) {
        return ShooterConstants.getHoodAngleForDistance(distanceMeters);
    }
    
    /**
     * Calculate flywheel RPM based on distance
     */
    public static double calculateFlywheelRPM(double distanceMeters) {
        return ShooterConstants.getFlywheelRPMForDistance(distanceMeters);
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
        
        double distance = calculateDistance(robotPose, targetPosition);
        double turretAngle = calculateTurretAngle(robotPose, targetPosition, turretOnLeft);
        boolean canReach = canTurretReachAngle(turretAngle, turretOnLeft);
        
        // Clamp to valid range if out of bounds
        double clampedAngle = clampTurretAngle(turretAngle, turretOnLeft);
        
        double hoodAngle = calculateHoodAngle(distance);
        double flywheelRPM = calculateFlywheelRPM(distance);
        
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
