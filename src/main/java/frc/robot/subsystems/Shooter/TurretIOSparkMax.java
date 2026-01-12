package frc.robot.subsystems.Shooter;

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
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ShooterConstants;

public class TurretIOSparkMax implements TurretIO {
    private final SparkMax turretMotor;
    // private final Servo hoodServo;
    private final SparkMax flywheelMotor;
    
    private final RelativeEncoder turretEncoder;
    private final RelativeEncoder flywheelEncoder;

    private final SparkClosedLoopController turretController;
    private final SparkClosedLoopController flywheelController;
    
    // Hood servo PWM channel
    // private static final int HOOD_SERVO_CHANNEL = ShooterConstants.kHoodServoChannel;
    
    // Track hood angle for logging
    private double hoodAngleDegrees = 0.0;
    
    // Track flywheel target for logging
    private double flywheelTargetRPM = 0.0;
    
    @SuppressWarnings("removal") // Suppress warnings for deprecated ResetMode and PersistMode
    public TurretIOSparkMax() {
        turretMotor = new SparkMax(ShooterConstants.kTurretMotorId, MotorType.kBrushless);
        // hoodServo = new Servo(HOOD_SERVO_CHANNEL);
        flywheelMotor = new SparkMax(ShooterConstants.kFlywheelMotorId, MotorType.kBrushless);
        
        // Get encoders from NEO motors
        turretEncoder = turretMotor.getEncoder();
        flywheelEncoder = flywheelMotor.getEncoder();

        turretController = turretMotor.getClosedLoopController();
        flywheelController = flywheelMotor.getClosedLoopController();
        
        // Configure motors
        turretMotor.configure(
            configureTurretMotor(), 
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
    
    private SparkMaxConfig configureFlywheelMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kCoast); // Coast for flywheel
        config.smartCurrentLimit(ShooterConstants.kFlywheelCurrentLimit);
        // Flywheel uses RPM directly
        config.encoder.velocityConversionFactor(1.0); // Already in RPM
        // Add PID gains for velocity control
        config.closedLoop.p(ShooterConstants.kFlywheelP);
        config.closedLoop.i(ShooterConstants.kFlywheelI);
        config.closedLoop.d(ShooterConstants.kFlywheelD);
        config.closedLoop.velocityFF(ShooterConstants.kFlywheelFF);
        config.closedLoop.outputRange(-1.0, 1.0);
        return config;
    }
    
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // Turret inputs
        inputs.turretVoltage = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();
        inputs.turretCurrent = turretMotor.getOutputCurrent();
        inputs.turretAngleDegrees = turretEncoder.getPosition();
        
        // Hood servo inputs
        // inputs.hoodVoltage = 0.0; // Servos don't report voltage
        // inputs.hoodCurrent = 0.0; // Servos don't report current
        inputs.hoodAngleDegrees = hoodAngleDegrees;
        
        // Flywheel inputs
        inputs.flywheelVoltage = flywheelMotor.getAppliedOutput() * flywheelMotor.getBusVoltage();
        inputs.flywheelCurrent = flywheelMotor.getOutputCurrent();
        inputs.flywheelVelocityRPM = flywheelEncoder.getVelocity();
        inputs.flywheelTargetRPM = flywheelTargetRPM;
    }
    
    @Override
    public void setTurretVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        turretMotor.setVoltage(clampedVoltage);
    }
    
    // @Override
    // public void setHoodVoltage(double voltage) {
    //     // Servos don't use voltage control - this is a no-op
    //     // Use setHoodAngle() instead
    // }
    
    @Override
    public void setFlywheelVoltage(double voltage) {
        flywheelTargetRPM = 0.0; // Clear velocity target when using voltage
        double clampedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        flywheelMotor.setVoltage(clampedVoltage);
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        flywheelTargetRPM = rpm;
        flywheelController.setSetpoint(rpm, ControlType.kVelocity);
    }

    @Override
    public void setTurretAngle(double angleDegrees) {
        turretController.setSetpoint(angleDegrees, ControlType.kPosition);
    }

    // @Override
    // public void setHoodAngle(double angleDegrees) {
    //     // Clamp to valid range
    //     hoodAngleDegrees = MathUtil.clamp(angleDegrees, 
    //         ShooterConstants.kHoodMinAngle, 
    //         ShooterConstants.kHoodMaxAngle);
        
    //     // Convert degrees to servo position (0.0 to 1.0)
    //     double range = ShooterConstants.kHoodMaxAngle - ShooterConstants.kHoodMinAngle;
    //     double position = (hoodAngleDegrees - ShooterConstants.kHoodMinAngle) / range;
    //     hoodServo.set(position);
    // }
    
    @Override
    public void stop() {
        flywheelTargetRPM = 0.0;
        turretMotor.stopMotor();
        flywheelMotor.stopMotor();
        // Servo holds position when stopped
    }
}
