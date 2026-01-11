package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

import frc.robot.Constants.ShooterConstants;

public class IndexerIOSparkMax implements IndexerIO {
    // Simulation Motor
    private final SparkMax indexerMotor;

    @SuppressWarnings("removal") // Suppress warnings for deprecated ResetMode and PersistMode
    public IndexerIOSparkMax() {
        indexerMotor = new SparkMax(ShooterConstants.kIndexerMotorId, SparkMax.MotorType.kBrushed);

        indexerMotor.configure(configureMotor(false), 
            SparkMax.ResetMode.kResetSafeParameters, 
            SparkMax.PersistMode.kPersistParameters);
    }

    private SparkMaxConfig configureMotor(boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted);
        config.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake); 
        config.smartCurrentLimit(ShooterConstants.kIndexerCurrentLimit);
        return config;
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Set voltage inputs
        inputs.indexerVoltage = indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage();
        inputs.indexerCurrent = indexerMotor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        indexerMotor.setVoltage(clampedVoltage);
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }
}
