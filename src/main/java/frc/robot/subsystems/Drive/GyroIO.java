package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public double pitchPositionRad = 0.0;
        public double pitchVelocityRadPerSec = 0.0;
        public double rollPositionRad = 0.0;
        public double rollVelocityRadPerSec = 0.0;
    }
    
    /** Updates the set of loggable inputs */
    public default void updateInputs(GyroIOInputs inputs) {}
    
    /** Reset the gyro yaw to zero */
    public default void reset() {}
    
    /** Set the gyro yaw to a specific angle */
    public default void setYaw(double angleDegrees) {}
}
