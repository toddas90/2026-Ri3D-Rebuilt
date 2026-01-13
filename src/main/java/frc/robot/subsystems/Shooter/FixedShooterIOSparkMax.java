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
import frc.robot.Constants.ShooterConstants;

public class FixedShooterIOSparkMax implements FixedShooterIO {
    private final SparkMax leftFlywheelMotor;
    private final SparkMax rightFlywheelMotor;
    
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    
    private final SparkClosedLoopController leftController;
    private final SparkClosedLoopController rightController;
    
    private double leftTargetRPM = 0.0;
    private double rightTargetRPM = 0.0;
    
    // Assuming you'll define these CAN IDs
    private static final int LEFT_FLYWHEEL_ID = ShooterConstants.kLeftFlywheelMotorId;
    private static final int RIGHT_FLYWHEEL_ID = ShooterConstants.kRightFlywheelMotorId;
    
    @SuppressWarnings("removal")
    public FixedShooterIOSparkMax() {
        leftFlywheelMotor = new SparkMax(LEFT_FLYWHEEL_ID, MotorType.kBrushless);
        rightFlywheelMotor = new SparkMax(RIGHT_FLYWHEEL_ID, MotorType.kBrushless);
        
        leftEncoder = leftFlywheelMotor.getEncoder();
        rightEncoder = rightFlywheelMotor.getEncoder();
        
        leftController = leftFlywheelMotor.getClosedLoopController();
        rightController = rightFlywheelMotor.getClosedLoopController();
        
        // Configure motors
        SparkMaxConfig leftConfig = configureFlywheelMotor();
        SparkMaxConfig rightConfig = configureFlywheelMotor();
        
        leftFlywheelMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFlywheelMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    private SparkMaxConfig configureFlywheelMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(ShooterConstants.kFlywheelCurrentLimit);
        config.inverted(true);
        
        config.encoder.velocityConversionFactor(1.0 / ShooterConstants.kFlywheelGearRatio);
        
        config.closedLoop.pid(ShooterConstants.kFlywheelP, ShooterConstants.kFlywheelI, ShooterConstants.kFlywheelD);
        config.closedLoop.velocityFF(ShooterConstants.kFlywheelFF);
        config.closedLoop.outputRange(-1, 1);
        
        return config;
    }
    
    @Override
    public void updateInputs(FixedShooterIOInputs inputs) {
        inputs.leftFlywheelVoltage = leftFlywheelMotor.getAppliedOutput() * leftFlywheelMotor.getBusVoltage();
        inputs.leftFlywheelCurrent = leftFlywheelMotor.getOutputCurrent();
        inputs.leftFlywheelVelocityRPM = leftEncoder.getVelocity();
        inputs.leftFlywheelTargetRPM = leftTargetRPM;
        
        inputs.rightFlywheelVoltage = rightFlywheelMotor.getAppliedOutput() * rightFlywheelMotor.getBusVoltage();
        inputs.rightFlywheelCurrent = rightFlywheelMotor.getOutputCurrent();
        inputs.rightFlywheelVelocityRPM = rightEncoder.getVelocity();
        inputs.rightFlywheelTargetRPM = rightTargetRPM;
    }
    
    @Override
    public void setLeftFlywheelVoltage(double voltage) {
        leftTargetRPM = 0;
        leftFlywheelMotor.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    }
    
    @Override
    public void setRightFlywheelVoltage(double voltage) {
        rightTargetRPM = 0;
        rightFlywheelMotor.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    }
    
    @Override
    public void setLeftFlywheelVelocity(double rpm) {
        leftTargetRPM = rpm;
        leftController.setSetpoint(rpm, ControlType.kVelocity);
    }
    
    @Override
    public void setRightFlywheelVelocity(double rpm) {
        rightTargetRPM = rpm;
        rightController.setSetpoint(rpm, ControlType.kVelocity);
    }
    
    @Override
    public void stop() {
        leftFlywheelMotor.stopMotor();
        rightFlywheelMotor.stopMotor();
        leftTargetRPM = 0;
        rightTargetRPM = 0;
    }
}