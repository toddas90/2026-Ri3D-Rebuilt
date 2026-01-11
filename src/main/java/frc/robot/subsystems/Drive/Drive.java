package frc.robot.subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Timer;

public class Drive extends SubsystemBase {
    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    
    private static final double MAX_VOLTAGE = DriveConstants.kMaxVoltage;
    
    // Add kinematics and pose estimator
    private final MecanumDriveKinematics kinematics;
    private final MecanumDrivePoseEstimator poseEstimator;
    private MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions();
    private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
    
    // Track time for velocity integration
    private double lastTime = 0;
    
    public Drive(DriveIO io, GyroIO gyroIO) {
        this.io = io;
        this.gyroIO = gyroIO;
        
        // Initialize kinematics with wheel locations (in meters)
        kinematics = new MecanumDriveKinematics(
            DriveConstants.kFrontLeftWheelOffset,
            DriveConstants.kFrontRightWheelOffset,
            DriveConstants.kRearLeftWheelOffset,
            DriveConstants.kRearRightWheelOffset
        );
        
        // Initialize pose estimator (replaces odometry for vision fusion)
        poseEstimator = new MecanumDrivePoseEstimator(
            kinematics,
            new Rotation2d(),
            wheelPositions,
            new Pose2d()
        );
        
        lastTime = Timer.getFPGATimestamp();
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
        
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        
        // Update simulated sensors if in simulation
        if (gyroIO instanceof GyroIOSim && currentSpeeds != null) {
            ((GyroIOSim) gyroIO).setYawVelocity(currentSpeeds.omegaRadiansPerSecond);
        }
        
        // Update pose estimation
        updateOdometry();
        
        // Log pose and velocity data
        Logger.recordOutput("Drive/Pose", getPose());
        Logger.recordOutput("Drive/FieldX", getPose().getX());
        Logger.recordOutput("Drive/FieldY", getPose().getY());
        Logger.recordOutput("Drive/FieldRotation", getPose().getRotation().getDegrees());
        Logger.recordOutput("Drive/VelocityX", currentSpeeds.vxMetersPerSecond);
        Logger.recordOutput("Drive/VelocityY", currentSpeeds.vyMetersPerSecond);
        Logger.recordOutput("Drive/VelocityOmega", currentSpeeds.omegaRadiansPerSecond);
    }
    
    private void updateOdometry() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        
        // Convert chassis speeds to individual wheel speeds using kinematics
        var wheelSpeeds = kinematics.toWheelSpeeds(currentSpeeds);
        
        // Update wheel positions by integrating individual wheel velocities
        wheelPositions = new MecanumDriveWheelPositions(
            wheelPositions.frontLeftMeters + wheelSpeeds.frontLeftMetersPerSecond * dt,
            wheelPositions.frontRightMeters + wheelSpeeds.frontRightMetersPerSecond * dt,
            wheelPositions.rearLeftMeters + wheelSpeeds.rearLeftMetersPerSecond * dt,
            wheelPositions.rearRightMeters + wheelSpeeds.rearRightMetersPerSecond * dt
        );
        
        // Update pose estimator with gyro angle and wheel positions
        poseEstimator.update(gyroInputs.yawPosition, wheelPositions);
    }
    
    /**
     * Add a vision measurement to the pose estimator
     * @param visionPose The pose measured by vision
     * @param timestamp The timestamp of the measurement
     * @param stdDevs Standard deviations for the measurement (x, y, theta)
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
    }
    
    /**
     * Get the current estimated pose of the robot
     * @return Current pose
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    
    /**
     * Reset pose estimator to a specific pose
     * @param pose The pose to reset to
     */
    public void resetPose(Pose2d pose) {
        wheelPositions = new MecanumDriveWheelPositions();
        poseEstimator.resetPosition(
            gyroInputs.yawPosition,
            wheelPositions,
            pose
        );
    }
    
    /**
     * Reset pose estimator to origin
     */
    public void resetPose() {
        resetPose(new Pose2d());
    }
    
    /**
     * Drive the robot using mecanum drive kinematics (robot-oriented)
     * @param xSpeed Speed in the x direction (-1 to 1)
     * @param ySpeed Speed in the y direction (-1 to 1)
     * @param rotation Rotation speed (-1 to 1)
     */
    public void drive(double xSpeed, double ySpeed, double rotation) {
        // Clamp inputs
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
        rotation = MathUtil.clamp(rotation, -1.0, 1.0);
        
        // Calculate mecanum drive wheel speeds
        MecanumDrive.WheelSpeeds wheelSpeeds = MecanumDrive.driveCartesianIK(xSpeed, ySpeed, rotation);
        
        // Convert to voltages and send to motors
        io.setVoltage(
            wheelSpeeds.frontLeft * MAX_VOLTAGE,
            wheelSpeeds.frontRight * MAX_VOLTAGE,
            wheelSpeeds.rearLeft * MAX_VOLTAGE,
            wheelSpeeds.rearRight * MAX_VOLTAGE
        );
        
        // Store current speeds for odometry estimation (convert to m/s)
        currentSpeeds = new ChassisSpeeds(
            xSpeed * DriveConstants.kMaxSpeedMetersPerSecond,
            ySpeed * DriveConstants.kMaxSpeedMetersPerSecond,
            rotation * DriveConstants.kMaxAngularSpeedRadiansPerSecond
        );
    }
    
    /**
     * Drive the robot using field-oriented mecanum drive
     * @param xSpeed Speed in the x direction (-1 to 1) relative to field
     * @param ySpeed Speed in the y direction (-1 to 1) relative to field
     * @param rotation Rotation speed (-1 to 1)
     */
    public void driveFieldOriented(double xSpeed, double ySpeed, double rotation) {
        // Clamp inputs
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
        rotation = MathUtil.clamp(rotation, -1.0, 1.0);
        
        // Calculate mecanum drive wheel speeds with field orientation
        MecanumDrive.WheelSpeeds wheelSpeeds = MecanumDrive.driveCartesianIK(
            xSpeed, 
            ySpeed, 
            rotation, 
            gyroInputs.yawPosition
        );
        
        // Convert to voltages and send to motors
        io.setVoltage(
            wheelSpeeds.frontLeft * MAX_VOLTAGE,
            wheelSpeeds.frontRight * MAX_VOLTAGE,
            wheelSpeeds.rearLeft * MAX_VOLTAGE,
            wheelSpeeds.rearRight * MAX_VOLTAGE
        );
        
        // Store field-relative speeds for odometry
        currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * DriveConstants.kMaxSpeedMetersPerSecond,
            ySpeed * DriveConstants.kMaxSpeedMetersPerSecond,
            rotation * DriveConstants.kMaxAngularSpeedRadiansPerSecond,
            gyroInputs.yawPosition
        );
    }
    
    /**
     * Stop all drive motors
     */
    public void stop() {
        io.stop();
        currentSpeeds = new ChassisSpeeds();
    }
    
    /**
     * Reset the gyro heading to zero
     */
    public void resetGyro() {
        gyroIO.reset();
    }
    
    /**
     * Get the current robot heading
     * @return Current heading as a Rotation2d
     */
    public Rotation2d getHeading() {
        return gyroInputs.yawPosition;
    }
    
    /**
     * Get the gyro yaw angular velocity
     * @return Angular velocity in radians per second
     */
    public double getAngularVelocity() {
        return gyroInputs.yawVelocityRadPerSec;
    }
    
    /**
     * Get the current estimated velocity
     * @return Current velocity as a ChassisSpeeds object
     */
    public ChassisSpeeds getVelocity() {
        return currentSpeeds;
    }
}
