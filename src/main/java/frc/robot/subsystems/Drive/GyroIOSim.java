package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class GyroIOSim implements GyroIO {
    private double yawPositionRad = 0.0;
    private double yawVelocityRadPerSec = 0.0;
    private double lastUpdateTime = 0.0;
    
    public GyroIOSim() {
        lastUpdateTime = Timer.getFPGATimestamp();
    }
    
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = new Rotation2d(yawPositionRad);
        inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
        
        // For simulation, pitch and roll remain at 0
        inputs.pitchPositionRad = 0.0;
        inputs.pitchVelocityRadPerSec = 0.0;
        inputs.rollPositionRad = 0.0;
        inputs.rollVelocityRadPerSec = 0.0;
    }
    
    @Override
    public void reset() {
        yawPositionRad = 0.0;
        yawVelocityRadPerSec = 0.0;
    }
    
    @Override
    public void setYaw(double angleDegrees) {
        yawPositionRad = Math.toRadians(angleDegrees);
    }
    
    /**
     * Update the simulated gyro with the commanded angular velocity
     * This should be called from the Drive subsystem in simulation
     */
    public void setYawVelocity(double velocityRadPerSec) {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;
        
        yawVelocityRadPerSec = velocityRadPerSec;
        yawPositionRad += velocityRadPerSec * dt;
    }
}
