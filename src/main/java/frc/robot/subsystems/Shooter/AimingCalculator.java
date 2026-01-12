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
    
    /**
     * Calculate complete aiming solution for moving robot
     */
    public static AimingParameters calculateAiming(
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Translation2d targetPosition,
            boolean isLeftTurret) {
        
        // Get turret position in 3D space
        Translation3d turretPos3d = isLeftTurret ? 
            ShooterConstants.kLeftTurretPosition : 
            ShooterConstants.kRightTurretPosition;
        
        // Convert to 2D field position
        Translation2d turretOffset2d = new Translation2d(turretPos3d.getX(), turretPos3d.getY());
        Translation2d rotatedOffset = turretOffset2d.rotateBy(robotPose.getRotation());
        Translation2d turretFieldPos = robotPose.getTranslation().plus(rotatedOffset);
        
        // Calculate initial distance to target
        double horizontalDistance = turretFieldPos.getDistance(targetPosition);
        double verticalDistance = ShooterConstants.HUB_HEIGHT - turretPos3d.getZ();
        
        // Find optimal trajectory (iterative approach for moving target)
        TrajectoryResult trajectory = findOptimalTrajectory(
            turretFieldPos, 
            targetPosition,
            turretPos3d.getZ(),
            ShooterConstants.HUB_HEIGHT,
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
        flywheelRPM = Math.min(flywheelRPM, ShooterConstants.kMaxFlywheelRPM);
        
        // Log results
        String side = isLeftTurret ? "Left" : "Right";
        Logger.recordOutput("Aiming/" + side + "/Distance_m", horizontalDistance); // Unit: meters
        Logger.recordOutput("Aiming/" + side + "/IdealHoodAngle", trajectory.hoodAngle); // Unit: degrees
        Logger.recordOutput("Aiming/" + side + "/LaunchVelocity_ms", trajectory.launchVelocity); // Unit: m/s
        Logger.recordOutput("Aiming/" + side + "/FlightTime", trajectory.flightTime); // Unit: s
        Logger.recordOutput("Aiming/" + side + "/IdealTurretAngle", turretAngle); // Unit: degrees
        Logger.recordOutput("Aiming/" + side + "/IdealFlywheel_RPM", flywheelRPM); // Unit: RPM
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
        double bestHoodAngle = ShooterConstants.kMinHoodAngle;
        double bestVelocity = Double.MAX_VALUE;
        double bestFlightTime = 0;
        boolean foundValidSolution = false;
        int validSolutionCount = 0;
        
        for (double hoodAngle = ShooterConstants.kMinHoodAngle; hoodAngle <= ShooterConstants.kMaxHoodAngle; hoodAngle += 1.0) {
            double angleRad = Math.toRadians(hoodAngle);
            
            // Solve projectile motion equation for initial velocity
            // y = x*tan(θ) - (g*x²)/(2*v₀²*cos²(θ))
            double cosAngle = Math.cos(angleRad);
            double tanAngle = Math.tan(angleRad);
            
            double denominator = 2 * cosAngle * cosAngle * (distance * tanAngle - heightDiff);
            if (denominator <= 0) continue;
            
            double velocitySquared = (ShooterConstants.kGravity * distance * distance) / denominator;
            if (velocitySquared < 0) continue;
            
            double requiredVelocity = Math.sqrt(velocitySquared);
            
            // Adjust for robot motion
            double launchVelocity = requiredVelocity - robotSpeed * Math.cos(angleRad);
            if (launchVelocity < 0) continue;
            
            // Check if achievable
            double rpm = velocityToRPM(launchVelocity / ShooterConstants.kLaunchEfficiency);
            if (rpm > ShooterConstants.kMaxFlywheelRPM) continue;
            
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
        Logger.recordOutput("Aiming/TrajectorySearch/SearchedAngles", (int)((ShooterConstants.kMaxHoodAngle - ShooterConstants.kMinHoodAngle) / 5.0) + 1);
        
        // If no solution found, use fallback
        if (!foundValidSolution) {
            Logger.recordOutput("Aiming/TrajectorySearch/FallbackReason", 
                distance < 2.0 ? "TooClose" : 
                distance > 5.0 ? "TooFar" : "NoPhysicsSolution");
            
            // Distance-based interpolation
            if (distance < 2.0) {
                bestHoodAngle = ShooterConstants.kMinHoodAngle;
            } else if (distance > 5.0) {
                bestHoodAngle = ShooterConstants.kMaxHoodAngle;
            } else {
                double t = (distance - 2.0) / 3.0;
                bestHoodAngle = ShooterConstants.kMinHoodAngle + t * (ShooterConstants.kMaxHoodAngle - ShooterConstants.kMinHoodAngle);
            }
            
            // Use max velocity
            bestVelocity = rpmToVelocity(ShooterConstants.kMaxFlywheelRPM) * ShooterConstants.kLaunchEfficiency;
            bestFlightTime = distance / (bestVelocity * Math.cos(Math.toRadians(bestHoodAngle)));
        }
        
        return new TrajectoryResult(
            bestHoodAngle,
            bestVelocity / ShooterConstants.kLaunchEfficiency, // Account for efficiency
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
        Translation3d turret3d = isLeftTurret ? ShooterConstants.kLeftTurretPosition : ShooterConstants.kRightTurretPosition;
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
            return !(normalizedAngle >= ShooterConstants.kLeftTurretDeadzoneMin && 
                    normalizedAngle <= ShooterConstants.kLeftTurretDeadzoneMax);
        } else {
            // Right turret cannot aim between 67.5° and 112.5° (directly left)
            return !(normalizedAngle >= ShooterConstants.kRightTurretDeadzoneMin && 
                    normalizedAngle <= ShooterConstants.kRightTurretDeadzoneMax);
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
            if (normalizedAngle >= ShooterConstants.kLeftTurretDeadzoneMin && 
                normalizedAngle <= ShooterConstants.kLeftTurretDeadzoneMax) {
                double distToMin = Math.abs(normalizedAngle - ShooterConstants.kLeftTurretDeadzoneMin);
                double distToMax = Math.abs(normalizedAngle - ShooterConstants.kLeftTurretDeadzoneMax);
                return distToMin < distToMax ? ShooterConstants.kLeftTurretDeadzoneMin - 0.1 : ShooterConstants.kLeftTurretDeadzoneMax + 0.1;
            }
        } else {
            // If in dead zone (67.5° to 112.5°), move to closest edge
            if (normalizedAngle >= ShooterConstants.kRightTurretDeadzoneMin && 
                normalizedAngle <= ShooterConstants.kRightTurretDeadzoneMax) {
                double distToMin = Math.abs(normalizedAngle - ShooterConstants.kRightTurretDeadzoneMin);
                double distToMax = Math.abs(normalizedAngle - ShooterConstants.kRightTurretDeadzoneMax);
                return distToMin < distToMax ? ShooterConstants.kRightTurretDeadzoneMin - 0.1 : ShooterConstants.kRightTurretDeadzoneMax + 0.1;
            }
        }
        return normalizedAngle;
    }
    
    /**
     * Convert velocity to flywheel RPM
     */
    private static double velocityToRPM(double velocity) {
        double angularVelocity = velocity / ShooterConstants.kFlywheelRadius;
        return angularVelocity * 60.0 / (2 * Math.PI);
    }
    
    /**
     * Convert RPM to exit velocity
     */
    private static double rpmToVelocity(double rpm) {
        double angularVelocity = rpm * 2 * Math.PI / 60.0;
        return angularVelocity * ShooterConstants.kFlywheelRadius;
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
