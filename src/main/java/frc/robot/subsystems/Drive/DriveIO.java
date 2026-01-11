package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public double frontLeftVoltage = 0.0;
        public double frontRightVoltage = 0.0;
        public double rearLeftVoltage = 0.0;
        public double rearRightVoltage = 0.0;
        
        public double frontLeftCurrent = 0.0;
        public double frontRightCurrent = 0.0;
        public double rearLeftCurrent = 0.0;
        public double rearRightCurrent = 0.0;
    }
    
    /** Updates the set of loggable inputs */
    public default void updateInputs(DriveIOInputs inputs) {}
    
    /** Run the drive motors at specified voltages */
    public default void setVoltage(double frontLeft, double frontRight, double rearLeft, double rearRight) {}
    
    /** Stop all motors */
    public default void stop() {}
}
