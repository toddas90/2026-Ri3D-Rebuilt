package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;

import frc.robot.Constants.ShooterConstants;

public class IndexerIOTalonSRX implements IndexerIO {
    private final WPI_TalonSRX indexerMotor;

    public IndexerIOTalonSRX() {
        indexerMotor = new WPI_TalonSRX(ShooterConstants.kIndexerMotorId);

        // Factory reset
        indexerMotor.configFactoryDefault();

        // Configure motor
        indexerMotor.setInverted(false);
        indexerMotor.setNeutralMode(NeutralMode.Brake);
        
        // Configure current limit
        indexerMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(
                true, 
                ShooterConstants.kIndexerCurrentLimit, 
                ShooterConstants.kIndexerCurrentLimit, 
                0.1
            )
        );
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerVoltage = indexerMotor.getMotorOutputVoltage();
        inputs.indexerCurrent = indexerMotor.getSupplyCurrent();
    }

    @Override
    public void setVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        indexerMotor.setVoltage(clampedVoltage);
    }

    @Override
    public void stop() {
        indexerMotor.stopMotor();
    }
}