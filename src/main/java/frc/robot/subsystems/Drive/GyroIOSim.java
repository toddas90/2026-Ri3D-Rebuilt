package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    private double yawPositionRad = 0.0;
    private double yawVelocityRadPerSec = 0.0;
    private double lastYawVelocityRadPerSec = 0.0;
    
    // Simulation constants
    private static final double LOOP_PERIOD_SECS = 0.02;
    
    public GyroIOSim() {
        // Initialize at zero
        reset();
    }
    
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // In simulation, we'll need to integrate angular velocity from the drive system
        // This is a simplified model - in reality you'd calculate this from wheel speeds
        yawPositionRad += yawVelocityRadPerSec * LOOP_PERIOD_SECS;
        
        inputs.connected = true; // Always connected in sim
        inputs.yawPosition = new Rotation2d(yawPositionRad);
        inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
        
        // Pitch and roll stay at zero in sim (robot on flat ground)
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
     * Set the angular velocity for simulation
     * This should be called by the drive subsystem based on wheel speeds
     */
    public void setAngularVelocity(double radPerSec) {
        yawVelocityRadPerSec = radPerSec;
    }
}
