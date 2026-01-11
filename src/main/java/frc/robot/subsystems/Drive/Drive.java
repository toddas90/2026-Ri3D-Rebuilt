package frc.robot.subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
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
    
    // Add kinematics and odometry
    private final MecanumDriveKinematics kinematics;
    private final MecanumDriveOdometry odometry;
    private MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions();
    private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
    
    // Track time for velocity integration (since no encoders)
    private double lastTime = 0;
    
    public Drive(DriveIO io, GyroIO gyroIO) {
        this.io = io;
        this.gyroIO = gyroIO;
        
        // Initialize kinematics with wheel locations (in meters)
        // Adjust these values based on your robot dimensions
        kinematics = new MecanumDriveKinematics(
            new Translation2d(0.3, 0.3),   // Front left
            new Translation2d(0.3, -0.3),  // Front right  
            new Translation2d(-0.3, 0.3),  // Rear left
            new Translation2d(-0.3, -0.3)  // Rear right
        );
        
        // Initialize odometry
        odometry = new MecanumDriveOdometry(
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
        
        // Update pose estimation
        updateOdometry();
        
        // Log pose
        Logger.recordOutput("Drive/Pose", getPose());
        Logger.recordOutput("Drive/FieldX", getPose().getX());
        Logger.recordOutput("Drive/FieldY", getPose().getY());
        Logger.recordOutput("Drive/FieldRotation", getPose().getRotation().getDegrees());
    }
    
    private void updateOdometry() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        
        // Since we don't have encoders, estimate wheel positions by integrating velocities
        // This is VERY approximate and will drift significantly
        wheelPositions = new MecanumDriveWheelPositions(
            wheelPositions.frontLeftMeters + currentSpeeds.vxMetersPerSecond * dt,
            wheelPositions.frontRightMeters + currentSpeeds.vxMetersPerSecond * dt,
            wheelPositions.rearLeftMeters + currentSpeeds.vxMetersPerSecond * dt,
            wheelPositions.rearRightMeters + currentSpeeds.vxMetersPerSecond * dt
        );
        
        // Update odometry with gyro angle and estimated wheel positions
        odometry.update(gyroInputs.yawPosition, wheelPositions);
    }
    
    /**
     * Get the current estimated pose of the robot
     * @return Current pose
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    
    /**
     * Reset odometry to a specific pose
     * @param pose The pose to reset to
     */
    public void resetPose(Pose2d pose) {
        wheelPositions = new MecanumDriveWheelPositions();
        odometry.resetPosition(
            gyroInputs.yawPosition,
            wheelPositions,
            pose
        );
    }
    
    /**
     * Reset odometry to origin
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
        MecanumDrive.WheelSpeeds wheelSpeeds = MecanumDrive.driveCartesianIK(ySpeed, xSpeed, rotation);
        
        // Convert to voltages and send to motors
        io.setVoltage(
            wheelSpeeds.frontLeft * MAX_VOLTAGE,
            wheelSpeeds.frontRight * MAX_VOLTAGE,
            wheelSpeeds.rearLeft * MAX_VOLTAGE,
            wheelSpeeds.rearRight * MAX_VOLTAGE
        );
        
        // Store current speeds for odometry estimation (convert to m/s)
        // Assuming max speed of ~3 m/s at full throttle (adjust based on your robot)
        currentSpeeds = new ChassisSpeeds(
            xSpeed * 3.0,  // Convert to m/s
            ySpeed * 3.0,  // Convert to m/s
            rotation * Math.PI * 2  // Convert to rad/s (assuming ~1 rotation per second at full)
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
            ySpeed, 
            xSpeed, 
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
            xSpeed * 3.0,  // Convert to m/s
            ySpeed * 3.0,  // Convert to m/s
            rotation * Math.PI * 2,  // Convert to rad/s
            gyroInputs.yawPosition
        );
    }
    
    /**
     * Stop all drive motors
     */
    public void stop() {
        io.stop();
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
}
