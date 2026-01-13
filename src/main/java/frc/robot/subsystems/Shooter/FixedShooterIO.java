package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FixedShooterIO {
    @AutoLog
    public static class FixedShooterIOInputs {
        public double leftFlywheelVoltage = 0.0;
        public double leftFlywheelCurrent = 0.0;
        public double leftFlywheelVelocityRPM = 0.0;
        public double leftFlywheelTargetRPM = 0.0;
        
        public double rightFlywheelVoltage = 0.0;
        public double rightFlywheelCurrent = 0.0;
        public double rightFlywheelVelocityRPM = 0.0;
        public double rightFlywheelTargetRPM = 0.0;
    }
    
    /** Updates the set of loggable inputs */
    public default void updateInputs(FixedShooterIOInputs inputs) {}
    
    /** Set flywheel voltages */
    public default void setLeftFlywheelVoltage(double voltage) {}
    public default void setRightFlywheelVoltage(double voltage) {}
    
    /** Set flywheel velocities in RPM */
    public default void setLeftFlywheelVelocity(double rpm) {}
    public default void setRightFlywheelVelocity(double rpm) {}
    
    /** Stop all motors */
    public default void stop() {}
}