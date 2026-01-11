package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
        public double flywheelVoltage = 0.0;
        public double flywheelCurrent = 0.0;
        public double turretAngleDegrees = 0.0;
        public double hoodAngleDegrees = 0.0;
        public double turretCurrent = 0.0;
        public double hoodCurrent = 0.0;
        public double turretVoltage = 0.0;
        public double hoodVoltage = 0.0;
        public double flywheelVelocityRPM = 0.0;  // Add this
  }

    /** Updates the set of loggable inputs */
    public default void updateInputs(TurretIOInputs inputs) {}
    
    /** Run the motor at specified voltage */
    public default void setFlywheelVoltage(double voltage) {}
    public default void setHoodVoltage(double voltage) {}
    public default void setTurretVoltage(double voltage) {}

    /** Stop motors */
    public default void stop() {}

    /** Set the turret motor  */
    public default void setTurretAngle(double angleDegrees) {}

    /** Set the hood angle */
    public default void setHoodAngle(double angleDegrees) {}
}
