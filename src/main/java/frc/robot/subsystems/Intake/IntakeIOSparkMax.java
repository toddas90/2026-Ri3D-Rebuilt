package frc.robot.subsystems.Intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {

    private final SparkMax armMotor;
    private final SparkMax rollerMotor;

    private final RelativeEncoder armEncoder;

    private final SparkClosedLoopController armPID;

    private Double armSetpointDegrees = 0.0;

    @SuppressWarnings("removal") // Suppress warnings for deprecated ResetMode and PersistMode
    public IntakeIOSparkMax() {
        // Initialize arm motor
        armMotor = new SparkMax(IntakeConstants.kArmMotorId, MotorType.kBrushed);
        armEncoder = armMotor.getEncoder();
        armPID = armMotor.getClosedLoopController();

        // Initialize roller motor
        rollerMotor = new SparkMax(IntakeConstants.kRollerMotorId, MotorType.kBrushed);

        // Configure motors
        armMotor.configure(configureArmMotor(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor.configure(configureRollerMotor(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private SparkMaxConfig configureArmMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(IntakeConstants.kArmCurrentLimit);
        config.voltageCompensation(12.0);

        // Configure PID
        config.closedLoop.pid(
            IntakeConstants.kArmP,
            IntakeConstants.kArmI,
            IntakeConstants.kArmD
        );

        // configure encoder conversion factors
        // convert from rotations to degrees 
        config.encoder.positionConversionFactor(IntakeConstants.kArmDegreesPerRotation);

        // Soft limits - set slightly beyond the actual positions to allow reaching them
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimit(IntakeConstants.kOutPosition + 5.0);  // Allow slight overshoot
        config.softLimit.reverseSoftLimit(IntakeConstants.kInPosition - 5.0);   // Allow slight undershoot

        return config;
    }

    private SparkMaxConfig configureRollerMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(IntakeConstants.kRollerCurrentLimit);
        config.voltageCompensation(12.0);

        return config;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Update arm inputs
        inputs.armPositionDegrees = armEncoder.getPosition();
        inputs.armAppliedCurrentAmps = armMotor.getOutputCurrent();
        inputs.armAppliedVolts = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
        inputs.armSetPointDegrees = armSetpointDegrees;

        // Update roller inputs
        inputs.rollerAppliedCurrentAmps = rollerMotor.getOutputCurrent();
        inputs.rollerAppliedVolts = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();
    }

    @Override
    public void setArmPosition(double positionDegrees) {
        double clampedPosition = MathUtil.clamp(
            positionDegrees, 
            IntakeConstants.kInPosition,
            IntakeConstants.kOutPosition
        );
        armSetpointDegrees = clampedPosition;
        armPID.setReference(clampedPosition, ControlType.kPosition);
    }

    @Override
    public void setRollerVoltage(double volatage) {
        double clampedVolts = MathUtil.clamp(volatage, -12.0, 12.0);
        rollerMotor.setVoltage(clampedVolts);
    }

    @Override
    public void stop() {
        armMotor.stopMotor();
        rollerMotor.stopMotor();
    }

    @Override
    public void resetEncoders() {
        armEncoder.setPosition(0.0);    
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        SparkMaxConfig armConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();

        IdleMode mode = enabled ? IdleMode.kBrake : IdleMode.kCoast;
        armConfig.idleMode(mode);
        rollerConfig.idleMode(mode);

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


}
