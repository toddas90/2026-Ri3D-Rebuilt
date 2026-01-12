package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        // Lift motor inputs
        public double liftPositionMeters = 0.0;
        public double liftVelocityMetersPerSec = 0.0;
        public double liftAppliedVolts = 0.0;
        public double liftCurrentAmps = 0.0;
        public double liftTempCelsius = 0.0;
        public double liftSetpointMeters = 0.0;
        
        // Pivot motor inputs
        public double pivotPositionDegrees = 0.0;
        public double pivotVelocityDegreesPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double pivotTempCelsius = 0.0;
        public double pivotSetpointDegrees = 0.0;
    }
    
    /** Updates the set of loggable inputs */
    public default void updateInputs(ClimbIOInputs inputs) {}
    
    /** Set lift position in meters */
    public default void setLiftPosition(double positionMeters) {}
    
    /** Set pivot angle in degrees */
    public default void setPivotAngle(double angleDegrees) {}
    
    /** Set lift motor voltage directly */
    public default void setLiftVoltage(double volts) {}
    
    /** Set pivot motor voltage directly */
    public default void setPivotVoltage(double volts) {}
    
    /** Stop all motors */
    public default void stop() {}
    
    /** Reset encoders to zero */
    public default void resetEncoders() {}
    
    /** Set whether brake mode is enabled */
    public default void setBrakeMode(boolean enabled) {}
}