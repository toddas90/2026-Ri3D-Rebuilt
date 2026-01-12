package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class AimingCalculator {
    
    // Physical Constants
    private static final double GRAVITY = 9.81; // m/s^2
    private static final double FLYWHEEL_RADIUS = 1.5 * 0.0254; // 1.5" radius in meters
    private static final double MAX_FLYWHEEL_RPM = 5600.0;
    private static final double LAUNCH_EFFICIENCY = 0.80; // Energy transfer efficiency
    
    // Hood angle limits
    private static final double MIN_HOOD_ANGLE = 45.0; // degrees - distance shot
    private static final double MAX_HOOD_ANGLE = 75.0; // degrees - upward shot
    
    // Target height (hub is 72" tall, opening on top)
    private static final double TARGET_HEIGHT = 72.0 * 0.0254; // 1.8288m
    
    // Turret mounting positions (from robot center)
    // Right turret: +14" forward (X), -8" right (Y), +14" up (Z)
    private static final Translation3d RIGHT_TURRET_POSITION = new Translation3d(
        8.0 * 0.0254,  // 0.3556m forward (X)
        -14.0 * 0.0254,  // 0.2032m right (-Y)
        14.0 * 0.0254   // 0.3556m up (Z)
    );
    
    // Left turret: +14" forward (X), +8" left (Y), +14" up (Z)
    private static final Translation3d LEFT_TURRET_POSITION = new Translation3d(
        8.0 * 0.0254,  // 0.3556m forward (X)
        14.0 * 0.0254,   // 0.2032m left (+Y)
        14.0 * 0.0254   // 0.3556m up (Z)
    );

    // Turret FOV limits (in degrees, 0° = robot forward, positive CCW)
    // Left turret: can aim anywhere except directly right (-90° ± 22.5°)
    private static final double LEFT_TURRET_DEADZONE_MIN = -112.5;  // -90° - 22.5°
    private static final double LEFT_TURRET_DEADZONE_MAX = -67.5;   // -90° + 22.5°
    
    // Right turret: can aim anywhere except directly left (90° ± 22.5°)  
    private static final double RIGHT_TURRET_DEADZONE_MIN = 67.5;   // 90° - 22.5°
    private static final double RIGHT_TURRET_DEADZONE_MAX = 112.5;  // 90° + 22.5°
    
    /**
     * Calculate complete aiming solution for moving robot
     */
    public static AimingParameters calculateAiming(
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Translation2d targetPosition,
            boolean isLeftTurret) {
        
        // Get turret position in 3D space
        Translation3d turretPos3d = isLeftTurret ? LEFT_TURRET_POSITION : RIGHT_TURRET_POSITION;
        
        // Convert to 2D field position
        Translation2d turretOffset2d = new Translation2d(turretPos3d.getX(), turretPos3d.getY());
        Translation2d rotatedOffset = turretOffset2d.rotateBy(robotPose.getRotation());
        Translation2d turretFieldPos = robotPose.getTranslation().plus(rotatedOffset);
        
        // Calculate initial distance to target
        double horizontalDistance = turretFieldPos.getDistance(targetPosition);
        double verticalDistance = TARGET_HEIGHT - turretPos3d.getZ();
        
        // Find optimal trajectory (iterative approach for moving target)
        TrajectoryResult trajectory = findOptimalTrajectory(
            turretFieldPos, 
            targetPosition,
            turretPos3d.getZ(),
            TARGET_HEIGHT,
            robotPose,
            robotVelocity
        );
        
        // Calculate turret angle accounting for projectile flight time
        double turretAngle = calculateTurretAngle(
            robotPose,
            robotVelocity,
            turretFieldPos,
            targetPosition,
            trajectory.flightTime,
            isLeftTurret
        );
        
        // Also calculate the current angle (without motion compensation) for reachability check
        Translation2d currentToTarget = targetPosition.minus(turretFieldPos);
        double currentFieldAngle = Math.atan2(currentToTarget.getY(), currentToTarget.getX());
        double currentRobotRelativeAngle = Math.toDegrees(currentFieldAngle - robotPose.getRotation().getRadians());
        
        // Normalize current angle to [-180, 180]
        while (currentRobotRelativeAngle > 180) currentRobotRelativeAngle -= 360;
        while (currentRobotRelativeAngle < -180) currentRobotRelativeAngle += 360;
        
        // Check if turret can reach the current angle (not the predicted one)
        boolean canReach = isAngleReachable(currentRobotRelativeAngle, isLeftTurret);
        if (!canReach) {
            // If we can't reach it now, clamp the turret angle but don't spin up
            turretAngle = clampTurretAngle(currentRobotRelativeAngle, isLeftTurret);
        }
        
        // Convert exit velocity to flywheel RPM
        double flywheelRPM = velocityToRPM(trajectory.launchVelocity);
        flywheelRPM = Math.min(flywheelRPM, MAX_FLYWHEEL_RPM);
        
        // Log results
        String side = isLeftTurret ? "Left" : "Right";
        Logger.recordOutput("Aiming/" + side + "/Distance_m", horizontalDistance); // Unit: meters
        Logger.recordOutput("Aiming/" + side + "/HoodAngle", trajectory.hoodAngle); // Unit: degrees
        Logger.recordOutput("Aiming/" + side + "/LaunchVelocity_ms", trajectory.launchVelocity); // Unit: m/s
        Logger.recordOutput("Aiming/" + side + "/FlightTime", trajectory.flightTime); // Unit: s
        Logger.recordOutput("Aiming/" + side + "/TurretAngle", turretAngle); // Unit: degrees
        Logger.recordOutput("Aiming/" + side + "/CurrentAngle", currentRobotRelativeAngle); // Unit: degrees
        Logger.recordOutput("Aiming/" + side + "/Flywheel_RPM", flywheelRPM); // Unit: RPM
        Logger.recordOutput("Aiming/" + side + "/CanReach", canReach); 
        Logger.recordOutput("Aiming/" + side + "/ValidSolutionFound", trajectory.isValidSolution);
        
        return new AimingParameters(
            turretAngle,
            trajectory.hoodAngle,
            flywheelRPM,
            horizontalDistance,
            canReach
        );
    }
    
    /**
     * Find optimal trajectory using iterative refinement
     */
    private static TrajectoryResult findOptimalTrajectory(
            Translation2d turretPosition,
            Translation2d targetPosition,
            double shooterHeight,
            double targetHeight,
            Pose2d robotPose,
            ChassisSpeeds robotVelocity) {
        
        double distance = turretPosition.getDistance(targetPosition);
        double heightDiff = targetHeight - shooterHeight;
        
        // Direction vector from turret to target (for velocity compensation)
        Translation2d toTarget = targetPosition.minus(turretPosition);
        double shootAngle = Math.atan2(toTarget.getY(), toTarget.getX());
        
        // Component of robot velocity in shooting direction
        double robotSpeed = robotVelocity.vxMetersPerSecond * Math.cos(shootAngle) +
                           robotVelocity.vyMetersPerSecond * Math.sin(shootAngle);
        
        // Try different hood angles to find optimal solution
        double bestHoodAngle = MIN_HOOD_ANGLE;
        double bestVelocity = Double.MAX_VALUE;
        double bestFlightTime = 0;
        boolean foundValidSolution = false;
        int validSolutionCount = 0;
        
        for (double hoodAngle = MIN_HOOD_ANGLE; hoodAngle <= MAX_HOOD_ANGLE; hoodAngle += 1.0) {
            double angleRad = Math.toRadians(hoodAngle);
            
            // Solve projectile motion equation for initial velocity
            // y = x*tan(θ) - (g*x²)/(2*v₀²*cos²(θ))
            double cosAngle = Math.cos(angleRad);
            double tanAngle = Math.tan(angleRad);
            
            double denominator = 2 * cosAngle * cosAngle * (distance * tanAngle - heightDiff);
            if (denominator <= 0) continue;
            
            double velocitySquared = (GRAVITY * distance * distance) / denominator;
            if (velocitySquared < 0) continue;
            
            double requiredVelocity = Math.sqrt(velocitySquared);
            
            // Adjust for robot motion
            double launchVelocity = requiredVelocity - robotSpeed * Math.cos(angleRad);
            if (launchVelocity < 0) continue;
            
            // Check if achievable
            double rpm = velocityToRPM(launchVelocity / LAUNCH_EFFICIENCY);
            if (rpm > MAX_FLYWHEEL_RPM) continue;
            
            // Calculate flight time
            double flightTime = distance / (requiredVelocity * cosAngle);
            
            // We found at least one valid solution
            validSolutionCount++;
            foundValidSolution = true;
            
            // Prefer minimum velocity (most efficient)
            if (launchVelocity < bestVelocity) {
                bestVelocity = launchVelocity;
                bestHoodAngle = hoodAngle;
                bestFlightTime = flightTime;
            }
        }
        
        // Log solution search results
        Logger.recordOutput("Aiming/TrajectorySearch/ValidSolutionsFound", validSolutionCount);
        Logger.recordOutput("Aiming/TrajectorySearch/SearchedAngles", (int)((MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) / 5.0) + 1);
        
        // If no solution found, use fallback
        if (!foundValidSolution) {
            Logger.recordOutput("Aiming/TrajectorySearch/FallbackReason", 
                distance < 2.0 ? "TooClose" : 
                distance > 5.0 ? "TooFar" : "NoPhysicsSolution");
            
            // Distance-based interpolation
            if (distance < 2.0) {
                bestHoodAngle = MIN_HOOD_ANGLE;
            } else if (distance > 5.0) {
                bestHoodAngle = MAX_HOOD_ANGLE;
            } else {
                double t = (distance - 2.0) / 3.0;
                bestHoodAngle = MIN_HOOD_ANGLE + t * (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE);
            }
            
            // Use max velocity
            bestVelocity = rpmToVelocity(MAX_FLYWHEEL_RPM) * LAUNCH_EFFICIENCY;
            bestFlightTime = distance / (bestVelocity * Math.cos(Math.toRadians(bestHoodAngle)));
        }
        
        return new TrajectoryResult(
            bestHoodAngle,
            bestVelocity / LAUNCH_EFFICIENCY, // Account for efficiency
            bestFlightTime,
            foundValidSolution
        );
    }
    
    /**
     * Calculate turret angle with lead compensation
     */
    private static double calculateTurretAngle(
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Translation2d turretPosition,
            Translation2d targetPosition,
            double flightTime,
            boolean isLeftTurret) {
        
        // Predict future robot position
        double futureX = robotPose.getX() + robotVelocity.vxMetersPerSecond * flightTime;
        double futureY = robotPose.getY() + robotVelocity.vyMetersPerSecond * flightTime;
        double futureHeading = robotPose.getRotation().getRadians() + 
                              robotVelocity.omegaRadiansPerSecond * flightTime;
        
        // Get turret offset and rotate to future orientation
        Translation3d turret3d = isLeftTurret ? LEFT_TURRET_POSITION : RIGHT_TURRET_POSITION;
        Translation2d turretOffset = new Translation2d(turret3d.getX(), turret3d.getY());
        Translation2d futureOffset = turretOffset.rotateBy(new Rotation2d(futureHeading));
        
        // Future turret position
        Translation2d futureTurretPos = new Translation2d(futureX, futureY).plus(futureOffset);
        
        // Calculate angle from future turret to target
        Translation2d toTarget = targetPosition.minus(futureTurretPos);
        double fieldAngle = Math.atan2(toTarget.getY(), toTarget.getX());
        
        // Convert to robot-relative angle
        double robotRelativeAngle = Math.toDegrees(fieldAngle - futureHeading);
        
        // Normalize to [-180, 180]
        while (robotRelativeAngle > 180) robotRelativeAngle -= 360;
        while (robotRelativeAngle < -180) robotRelativeAngle += 360;
        
        return robotRelativeAngle;
    }
    
    /**
     * Check if turret can reach the calculated angle
     */
    private static boolean isAngleReachable(double angle, boolean isLeftTurret) {
        // Normalize angle to [-180, 180]
        double normalizedAngle = angle;
        while (normalizedAngle > 180) normalizedAngle -= 360;
        while (normalizedAngle < -180) normalizedAngle += 360;
        
        if (isLeftTurret) {
            // Left turret cannot aim between -112.5° and -67.5° (directly right)
            return !(normalizedAngle >= LEFT_TURRET_DEADZONE_MIN && 
                    normalizedAngle <= LEFT_TURRET_DEADZONE_MAX);
        } else {
            // Right turret cannot aim between 67.5° and 112.5° (directly left)
            return !(normalizedAngle >= RIGHT_TURRET_DEADZONE_MIN && 
                    normalizedAngle <= RIGHT_TURRET_DEADZONE_MAX);
        }
    }
    
    /**
     * Clamp turret angle to valid range
     */
    private static double clampTurretAngle(double angle, boolean isLeftTurret) {
        // Normalize angle to [-180, 180]
        double normalizedAngle = angle;
        while (normalizedAngle > 180) normalizedAngle -= 360;
        while (normalizedAngle < -180) normalizedAngle += 360;
        
        if (isLeftTurret) {
            // If in dead zone (-112.5° to -67.5°), move to closest edge
            if (normalizedAngle >= LEFT_TURRET_DEADZONE_MIN && 
                normalizedAngle <= LEFT_TURRET_DEADZONE_MAX) {
                double distToMin = Math.abs(normalizedAngle - LEFT_TURRET_DEADZONE_MIN);
                double distToMax = Math.abs(normalizedAngle - LEFT_TURRET_DEADZONE_MAX);
                return distToMin < distToMax ? LEFT_TURRET_DEADZONE_MIN - 0.1 : LEFT_TURRET_DEADZONE_MAX + 0.1;
            }
        } else {
            // If in dead zone (67.5° to 112.5°), move to closest edge
            if (normalizedAngle >= RIGHT_TURRET_DEADZONE_MIN && 
                normalizedAngle <= RIGHT_TURRET_DEADZONE_MAX) {
                double distToMin = Math.abs(normalizedAngle - RIGHT_TURRET_DEADZONE_MIN);
                double distToMax = Math.abs(normalizedAngle - RIGHT_TURRET_DEADZONE_MAX);
                return distToMin < distToMax ? RIGHT_TURRET_DEADZONE_MIN - 0.1 : RIGHT_TURRET_DEADZONE_MAX + 0.1;
            }
        }
        return normalizedAngle;
    }
    
    /**
     * Convert velocity to flywheel RPM
     */
    private static double velocityToRPM(double velocity) {
        double angularVelocity = velocity / FLYWHEEL_RADIUS;
        return angularVelocity * 60.0 / (2 * Math.PI);
    }
    
    /**
     * Convert RPM to exit velocity
     */
    private static double rpmToVelocity(double rpm) {
        double angularVelocity = rpm * 2 * Math.PI / 60.0;
        return angularVelocity * FLYWHEEL_RADIUS;
    }
    
    /**
     * Get target tower position based on alliance
     */
    public static Translation2d getTargetTowerPosition() {
        var alliance = DriverStation.getAlliance();
        boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Blue;
        return isBlue ? ShooterConstants.BLUE_HUB_POSITION : ShooterConstants.RED_HUB_POSITION;
    }
    
    /**
     * Get driver station position for shooting back
     */
    public static Translation2d getDriverStationPosition() {
        var alliance = DriverStation.getAlliance();
        boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Blue;
        return isBlue ? ShooterConstants.BLUE_DRIVER_STATION : ShooterConstants.RED_DRIVER_STATION;
    }
    
    // Helper classes
    private static record TrajectoryResult(
        double hoodAngle,
        double launchVelocity,
        double flightTime,
        boolean isValidSolution
    ) {}
    
    public static record AimingParameters(
        double turretAngleDegrees,
        double hoodAngleDegrees,
        double flywheelRPM,
        double distanceMeters,
        boolean canReachTarget
    ) {}
}
