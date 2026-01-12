package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private static final double MAX_VOLTAGE = DriveConstants.kMaxVoltage;

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Indexer", inputs);
        
        // Log data
        Logger.recordOutput("Shooter/Indexer/AppliedVoltage", inputs.indexerVoltage);
        Logger.recordOutput("Shooter/Indexer/Current", inputs.indexerCurrent);
    }

    // ==================== Indexer Control ====================
    /**
     * Start indexer
     */
    public void start() {
        io.setVoltage(MAX_VOLTAGE);
    }

    /**
     * Stop indexer
     */
    public void stop() {
        io.stop();
    }

    public boolean isRunning() {
        return inputs.indexerCurrent > 0.1;
    }
}
