package frc.robot.subsystems.Climb;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOSparkMax implements ClimbIO {
    private final SparkMax liftMotor;
    private final SparkMax pivotMotor;
    
    private final RelativeEncoder liftEncoder;
    private final RelativeEncoder pivotEncoder;
    
    private final SparkClosedLoopController liftPID;
    private final SparkClosedLoopController pivotPID;
    
    // Control mode tracking
    private boolean liftPositionMode = false;
    private boolean pivotPositionMode = false;

    // Track setpoints
    private double liftSetpointMeters = 0.0;
    private double pivotSetpointDegrees = 0.0;

    @SuppressWarnings("removal") // Suppress warnings for deprecated ResetMode and PersistMode
    public ClimbIOSparkMax() {
        // Initialize lift motor
        liftMotor = new SparkMax(ClimbConstants.kLiftMotorId, MotorType.kBrushless);
        liftEncoder = liftMotor.getEncoder();
        liftPID = liftMotor.getClosedLoopController();
        
        // Initialize pivot motor
        pivotMotor = new SparkMax(ClimbConstants.kPivotMotorId, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPID = pivotMotor.getClosedLoopController();
        
        // Configure motors
        configureLiftMotor();
        configurePivotMotor();
        
        // Apply configurations
        liftMotor.configure(configureLiftMotor(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.configure(configurePivotMotor(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    private SparkMaxConfig configureLiftMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(ClimbConstants.kLiftCurrentLimit);
        config.voltageCompensation(12.0);
        
        // Configure PID
        config.closedLoop.pidf(
            ClimbConstants.kLiftP,
            ClimbConstants.kLiftI,
            ClimbConstants.kLiftD,
            ClimbConstants.kLiftFF
        );
        
        // Configure encoder conversion factors
        // Convert from rotations to meters
        config.encoder.positionConversionFactor(ClimbConstants.kLiftMetersPerRotation);
        config.encoder.velocityConversionFactor(ClimbConstants.kLiftMetersPerRotation / 60.0);
        
        // Soft limits
        config.softLimit.forwardSoftLimit(ClimbConstants.kLiftMaxHeight);
        config.softLimit.reverseSoftLimit(ClimbConstants.kLiftMinHeight);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimitEnabled(true);
        
        return config;
    }
    
    private SparkMaxConfig configurePivotMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(ClimbConstants.kPivotCurrentLimit);
        config.voltageCompensation(12.0);
        
        // Configure PID
        config.closedLoop.pidf(
            ClimbConstants.kPivotP,
            ClimbConstants.kPivotI,
            ClimbConstants.kPivotD,
            ClimbConstants.kPivotFF
        );
        
        // Configure encoder conversion factors
        // Convert from rotations to degrees
        config.encoder.positionConversionFactor(ClimbConstants.kPivotDegreesPerRotation);
        config.encoder.velocityConversionFactor(ClimbConstants.kPivotDegreesPerRotation / 60.0);
        
        // Soft limits (0 to 180 degrees for full flip)
        config.softLimit.forwardSoftLimit(ClimbConstants.kPivotMaxAngle);
        config.softLimit.reverseSoftLimit(ClimbConstants.kPivotMinAngle);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimitEnabled(true);
        
        return config;
    }
    
    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        // Lift inputs
        inputs.liftPositionMeters = liftEncoder.getPosition();
        inputs.liftVelocityMetersPerSec = liftEncoder.getVelocity();
        inputs.liftAppliedVolts = liftMotor.getAppliedOutput() * liftMotor.getBusVoltage();
        inputs.liftCurrentAmps = liftMotor.getOutputCurrent();
        inputs.liftTempCelsius = liftMotor.getMotorTemperature();
        
        // Pivot inputs
        inputs.pivotPositionDegrees = pivotEncoder.getPosition();
        inputs.pivotVelocityDegreesPerSec = pivotEncoder.getVelocity();
        inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();
        inputs.pivotTempCelsius = pivotMotor.getMotorTemperature();
        
        // Add setpoint tracking
        inputs.liftSetpointMeters = liftPositionMode ? liftSetpointMeters : inputs.liftPositionMeters;
        inputs.pivotSetpointDegrees = pivotPositionMode ? pivotSetpointDegrees : inputs.pivotPositionDegrees;
    }
    
    @Override
    public void setLiftPosition(double positionMeters) {
        liftPositionMode = true;
        double clampedPosition = MathUtil.clamp(
            positionMeters, 
            ClimbConstants.kLiftMinHeight, 
            ClimbConstants.kLiftMaxHeight
        );
        liftSetpointMeters = clampedPosition;  // Track the setpoint
        liftPID.setReference(clampedPosition, ControlType.kPosition);
    }
    
    @Override
    public void setPivotAngle(double angleDegrees) {
        pivotPositionMode = true;
        double clampedAngle = MathUtil.clamp(
            angleDegrees,
            ClimbConstants.kPivotMinAngle,
            ClimbConstants.kPivotMaxAngle
        );
        pivotSetpointDegrees = clampedAngle;  // Track the setpoint
        pivotPID.setReference(clampedAngle, ControlType.kPosition);
    }
    
    @Override
    public void setLiftVoltage(double volts) {
        liftPositionMode = false;
        liftMotor.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }
    
    @Override
    public void setPivotVoltage(double volts) {
        pivotPositionMode = false;
        pivotMotor.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }
    
    @Override
    public void stop() {
        liftPositionMode = false;
        pivotPositionMode = false;
        liftMotor.stopMotor();
        pivotMotor.stopMotor();
    }
    
    @Override
    public void resetEncoders() {
        liftEncoder.setPosition(0.0);
        pivotEncoder.setPosition(0.0);
    }
    
    @SuppressWarnings("removal") // Suppress warnings for deprecated ResetMode and PersistMode
    @Override
    public void setBrakeMode(boolean enabled) {
        SparkMaxConfig liftConfig = new SparkMaxConfig();
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        
        IdleMode mode = enabled ? IdleMode.kBrake : IdleMode.kCoast;
        liftConfig.idleMode(mode);
        pivotConfig.idleMode(mode);
        
        liftMotor.configure(liftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}