package frc.robot.subsystems.Shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ShooterConstants;

public class TurretIOSparkMax implements TurretIO {
    private final SparkMax turretMotor;
    private final SparkMax hoodMotor;
    private final SparkMax flywheelMotor;
    
    private final RelativeEncoder turretEncoder;
    private final RelativeEncoder hoodEncoder;
    private final RelativeEncoder flywheelEncoder;

    private final SparkClosedLoopController turretController;
    private final SparkClosedLoopController hoodController;
    
    @SuppressWarnings("removal") // Suppress warnings for deprecated ResetMode and PersistMode
    public TurretIOSparkMax() {
        turretMotor = new SparkMax(ShooterConstants.kTurretMotorId, MotorType.kBrushless);
        hoodMotor = new SparkMax(ShooterConstants.kHoodMotorId, MotorType.kBrushless);
        flywheelMotor = new SparkMax(ShooterConstants.kFlywheelMotorId, MotorType.kBrushless);
        
        // Get encoders from NEO motors
        turretEncoder = turretMotor.getEncoder();
        hoodEncoder = hoodMotor.getEncoder();
        flywheelEncoder = flywheelMotor.getEncoder();

        turretController = turretMotor.getClosedLoopController();
        hoodController = hoodMotor.getClosedLoopController();
        
        // Configure motors
        turretMotor.configure(
            configureTurretMotor(), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
        hoodMotor.configure(
            configureHoodMotor(), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
        flywheelMotor.configure(
            configureFlywheelMotor(), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
    }
    
    private SparkMaxConfig configureTurretMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(ShooterConstants.kTurretCurrentLimit);
        // Configure encoder conversion factor
        config.encoder.positionConversionFactor(ShooterConstants.kTurretDegreesPerRotation);
        config.encoder.velocityConversionFactor(ShooterConstants.kTurretDegreesPerRotation / 60.0);
        // Add PID gains for position control
        config.closedLoop.p(ShooterConstants.kTurretP);
        config.closedLoop.i(ShooterConstants.kTurretI);
        config.closedLoop.d(ShooterConstants.kTurretD);
        config.closedLoop.outputRange(-1.0, 1.0);
        return config;
    }
    
    private SparkMaxConfig configureHoodMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(ShooterConstants.kHoodCurrentLimit);
        // Configure encoder conversion factor
        config.encoder.positionConversionFactor(ShooterConstants.kHoodDegreesPerRotation);
        config.encoder.velocityConversionFactor(ShooterConstants.kHoodDegreesPerRotation / 60.0);
        // Add PID gains for position control
        config.closedLoop.p(ShooterConstants.kHoodP);
        config.closedLoop.i(ShooterConstants.kHoodI);
        config.closedLoop.d(ShooterConstants.kHoodD);
        config.closedLoop.outputRange(-1.0, 1.0);
        return config;
    }
    
    private SparkMaxConfig configureFlywheelMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kCoast); // Coast for flywheel
        config.smartCurrentLimit(ShooterConstants.kFlywheelCurrentLimit);
        // Flywheel uses RPM directly (no gear ratio conversion needed for velocity)
        return config;
    }
    
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // Turret inputs
        inputs.turretVoltage = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();
        inputs.turretCurrent = turretMotor.getOutputCurrent();
        inputs.turretAngleDegrees = turretEncoder.getPosition();
        
        // Hood inputs
        inputs.hoodVoltage = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
        inputs.hoodCurrent = hoodMotor.getOutputCurrent();
        inputs.hoodAngleDegrees = hoodEncoder.getPosition();
        
        // Flywheel inputs
        inputs.flywheelVoltage = flywheelMotor.getAppliedOutput() * flywheelMotor.getBusVoltage();
        inputs.flywheelCurrent = flywheelMotor.getOutputCurrent();
        inputs.flywheelVelocityRPM = flywheelEncoder.getVelocity();
    }
    
    @Override
    public void setTurretVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        turretMotor.setVoltage(clampedVoltage);
    }
    
    @Override
    public void setHoodVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        hoodMotor.setVoltage(clampedVoltage);
    }
    
    @Override
    public void setFlywheelVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        flywheelMotor.setVoltage(clampedVoltage);
    }

    @Override
    public void setTurretAngle(double angleDegrees) {
        turretController.setSetpoint(angleDegrees, 
            ControlType.kPosition);
    }

    @Override
    public void setHoodAngle(double angleDegrees) {
        hoodController.setSetpoint(angleDegrees, 
            ControlType.kPosition);
    }
    
    @Override
    public void stop() {
        turretMotor.stopMotor();
        hoodMotor.stopMotor();
        flywheelMotor.stopMotor();
    }
}
