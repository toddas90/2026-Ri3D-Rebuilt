package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        //arm motor inputs 
        public double armPositionDegrees = 0.0; //uses radians internally
        public double armAppliedCurrentAmps = 0.0;
        public double armAppliedVolts = 0.0;
        public double armSetPointDegrees = 0.0;

        //roller motor inputs
        public double rollerAppliedCurrentAmps = 0.0;
        public double rollerAppliedVolts = 0.0;
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /** Set arm position in degrees */
    public default void setArmPosition(double positionDegrees) {}

    /** Set roller motor voltage directly */
    public default void setRollerVoltage(double volts) {}

    /** Stop all motors */
    public default void stop() {}

    /** Reset encoders to zero */
    public default void resetEncoders() {}

    /** Set whether brake mode is enabled */
    public default void setBrakeMode(boolean enabled) {}
}
