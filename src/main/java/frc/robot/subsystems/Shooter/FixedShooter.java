package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ShooterConstants;

public class FixedShooter extends SubsystemBase {
    private final FixedShooterIO io;
    private final FixedShooterIOInputsAutoLogged inputs = new FixedShooterIOInputsAutoLogged();
    
    // Fixed hood angle for simplified shooter
    private static final double FIXED_HOOD_ANGLE = 75.0; // degrees
    private static final double MAX_VOLTAGE = 12.0;
    
    public FixedShooter(FixedShooterIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("FixedShooter", inputs);
        
        // Log data
        Logger.recordOutput("FixedShooter/LeftFlywheelRPM", inputs.leftFlywheelVelocityRPM);
        Logger.recordOutput("FixedShooter/RightFlywheelRPM", inputs.rightFlywheelVelocityRPM);
        Logger.recordOutput("FixedShooter/LeftTargetRPM", inputs.leftFlywheelTargetRPM);
        Logger.recordOutput("FixedShooter/RightTargetRPM", inputs.rightFlywheelTargetRPM);
        Logger.recordOutput("FixedShooter/FixedHoodAngle", FIXED_HOOD_ANGLE);
    }
    
    /**
     * Set both flywheels to the same velocity
     * @param rpm Target velocity in RPM
     */
    public void setFlywheelVelocity(double rpm) {
        double clampedRPM = MathUtil.clamp(rpm, 0, ShooterConstants.kMaxFlywheelRPM);
        io.setLeftFlywheelVelocity(clampedRPM);
        io.setRightFlywheelVelocity(clampedRPM);
    }
    
    /**
     * Set flywheel velocities independently (for spin control)
     * @param leftRPM Left flywheel target RPM
     * @param rightRPM Right flywheel target RPM
     */
    public void setFlywheelVelocities(double leftRPM, double rightRPM) {
        io.setLeftFlywheelVelocity(MathUtil.clamp(leftRPM, 0, ShooterConstants.kMaxFlywheelRPM));
        io.setRightFlywheelVelocity(MathUtil.clamp(rightRPM, 0, ShooterConstants.kMaxFlywheelRPM));
    }
    
    /**
     * Set both flywheels to the same voltage
     * @param voltage Voltage to apply (-12 to 12)
     */
    public void setFlywheelVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
        io.setLeftFlywheelVoltage(clampedVoltage);
        io.setRightFlywheelVoltage(clampedVoltage);
    }
    
    /**
     * Check if flywheels are at target speed
     * @param toleranceRPM Acceptable tolerance
     * @return True if both flywheels are within tolerance
     */
    public boolean areFlywheelsAtSpeed(double toleranceRPM) {
        boolean leftAtSpeed = Math.abs(inputs.leftFlywheelVelocityRPM - inputs.leftFlywheelTargetRPM) < toleranceRPM;
        boolean rightAtSpeed = Math.abs(inputs.rightFlywheelVelocityRPM - inputs.rightFlywheelTargetRPM) < toleranceRPM;
        return leftAtSpeed && rightAtSpeed;
    }
    
    /**
     * Get the fixed hood angle
     * @return Hood angle in degrees
     */
    public double getHoodAngle() {
        return FIXED_HOOD_ANGLE;
    }
    
    /**
     * Stop all motors
     */
    public void stop() {
        io.stop();
    }
}