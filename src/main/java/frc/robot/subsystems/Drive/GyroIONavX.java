package frc.robot.subsystems.Drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIONavX implements GyroIO {
    private final AHRS navx;
    
    public GyroIONavX() {
        // Initialize navX on MXP port (SPI)
        navx = new AHRS(NavXComType.kMXP_SPI);
        navx.reset();
    }
    
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        
        // navX returns yaw in degrees, convert to Rotation2d
        // Negate because navX is CW positive, WPILib is CCW positive
        inputs.yawPosition = Rotation2d.fromDegrees(-navx.getYaw());
        
        // Convert angular velocity from degrees/sec to radians/sec
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navx.getRate());
        
        // Pitch and Roll in radians
        inputs.pitchPositionRad = Units.degreesToRadians(navx.getPitch());
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(navx.getRawGyroY());
        
        inputs.rollPositionRad = Units.degreesToRadians(navx.getRoll());
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(navx.getRawGyroX());
    }
    
    @Override
    public void reset() {
        navx.reset();
    }
    
    @Override
    public void setYaw(double angleDegrees) {
        // navX doesn't support setting yaw directly, so we offset instead
        navx.reset();
        navx.setAngleAdjustment(-angleDegrees);
    }
}
