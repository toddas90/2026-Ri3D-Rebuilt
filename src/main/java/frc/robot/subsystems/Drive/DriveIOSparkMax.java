package frc.robot.subsystems.Drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.DriveConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class DriveIOSparkMax implements DriveIO {
    private final SparkMax frontLeft;
    private final SparkMax frontRight;
    private final SparkMax rearLeft;
    private final SparkMax rearRight;
    
    // TODO: Update these CAN IDs
    private static final int FRONT_LEFT_ID = DriveConstants.kFrontLeftMotorId;
    private static final int FRONT_RIGHT_ID = DriveConstants.kFrontRightMotorId;
    private static final int REAR_LEFT_ID = DriveConstants.kRearLeftMotorId;
    private static final int REAR_RIGHT_ID = DriveConstants.kRearRightMotorId;
    
    @SuppressWarnings("removal") // Surpress warnings for deprecated ResetMode and PersistMode usage
    public DriveIOSparkMax() {
        frontLeft = new SparkMax(FRONT_LEFT_ID, MotorType.kBrushless);
        frontRight = new SparkMax(FRONT_RIGHT_ID, MotorType.kBrushless);
        rearLeft = new SparkMax(REAR_LEFT_ID, MotorType.kBrushless);
        rearRight = new SparkMax(REAR_RIGHT_ID, MotorType.kBrushless);
        
        // Configure motors
        frontLeft.configure(configureMotor(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        frontRight.configure(configureMotor(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rearLeft.configure(configureMotor(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rearRight.configure(configureMotor(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    private SparkMaxConfig configureMotor(boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(DriveConstants.kCurrentLimit);
        return config;
    }
    
    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.frontLeftVoltage = frontLeft.getAppliedOutput() * frontLeft.getBusVoltage();
        inputs.frontRightVoltage = frontRight.getAppliedOutput() * frontRight.getBusVoltage();
        inputs.rearLeftVoltage = rearLeft.getAppliedOutput() * rearLeft.getBusVoltage();
        inputs.rearRightVoltage = rearRight.getAppliedOutput() * rearRight.getBusVoltage();
        
        inputs.frontLeftCurrent = frontLeft.getOutputCurrent();
        inputs.frontRightCurrent = frontRight.getOutputCurrent();
        inputs.rearLeftCurrent = rearLeft.getOutputCurrent();
        inputs.rearRightCurrent = rearRight.getOutputCurrent();
    }
    
    @Override
    public void setVoltage(double frontLeftVolts, double frontRightVolts, 
                           double rearLeftVolts, double rearRightVolts) {
        frontLeft.setVoltage(frontLeftVolts);
        frontRight.setVoltage(frontRightVolts);
        rearLeft.setVoltage(rearLeftVolts);
        rearRight.setVoltage(rearRightVolts);
    }
    
    @Override
    public void stop() {
        frontLeft.stopMotor();
        frontRight.stopMotor();
        rearLeft.stopMotor();
        rearRight.stopMotor();
    }
}
