package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
        public double indexerVoltage = 0.0;
        public double indexerCurrent = 0.0;
  }

    /** Updates the set of loggable inputs */
    public default void updateInputs(IndexerIOInputs inputs) {}
    
    /** Run the motor at specified voltages */
    public default void setVoltage(double voltage) {}
    
    /** Stop motor */
    public default void stop() {}
}
