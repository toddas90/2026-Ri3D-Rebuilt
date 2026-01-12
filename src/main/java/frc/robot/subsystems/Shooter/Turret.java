package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final String name;

    private static final double MAX_VOLTAGE = 12.0;

    public Turret(TurretIO io, String name) {
        this.io = io;
        this.name = name;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/" + name, inputs);
        
        // Log data
        Logger.recordOutput("Shooter/" + name + "/FlywheelAppliedVoltage", inputs.flywheelVoltage);
        Logger.recordOutput("Shooter/" + name + "/FlywheelCurrent", inputs.flywheelCurrent);
        Logger.recordOutput("Shooter/" + name + "/TurretAngleDegrees", inputs.turretAngleDegrees);
        Logger.recordOutput("Shooter/" + name + "/HoodAngleDegrees", inputs.hoodAngleDegrees);
        Logger.recordOutput("Shooter/" + name + "/TurretAppliedVoltage", inputs.turretVoltage);
        Logger.recordOutput("Shooter/" + name + "/TurretCurrent", inputs.turretCurrent);
        Logger.recordOutput("Shooter/" + name + "/HoodAppliedVoltage", inputs.hoodVoltage);
        Logger.recordOutput("Shooter/" + name + "/HoodCurrent", inputs.hoodCurrent);
        
        // Log additional data
        Logger.recordOutput("Shooter/" + name + "/FlywheelVelocityRPM", inputs.flywheelVelocityRPM);
        Logger.recordOutput("Shooter/" + name + "/FlywheelTargetRPM", inputs.flywheelTargetRPM);
    }

    // ==================== Turret Control ====================
    
    /**
     * Set the turret to a specific angle
     * @param angleDegrees Target angle in degrees
     */
    public void setTurretAngle(double angleDegrees) {
        double clampedAngle = MathUtil.clamp(angleDegrees, 
            ShooterConstants.kTurretMinAngle, ShooterConstants.kTurretMaxAngle);
        io.setTurretAngle(clampedAngle);
    }
    
    /**
     * Set turret voltage directly (for manual control)
     * @param voltage Voltage to apply (-12 to 12)
     */
    public void setTurretVoltage(double voltage) {
        io.setTurretVoltage(voltage);
    }
    
    /**
     * Get the current turret angle
     * @return Turret angle in degrees
     */
    public double getTurretAngle() {
        return inputs.turretAngleDegrees;
    }
    
    // ==================== Hood Control ====================
    
    /**
     * Set the hood to a specific angle
     * @param angleDegrees Target angle in degrees
     */
    public void setHoodAngle(double angleDegrees) {
        double clampedAngle = MathUtil.clamp(angleDegrees, 
            ShooterConstants.kHoodMinAngle, ShooterConstants.kHoodMaxAngle);
        io.setHoodAngle(clampedAngle);
    }
    
    /**
     * Set hood voltage directly (for manual control)
     * @param voltage Voltage to apply (-12 to 12)
     */
    public void setHoodVoltage(double voltage) {
        io.setHoodVoltage(voltage);
    }
    
    /**
     * Get the current hood angle
     * @return Hood angle in degrees
     */
    public double getHoodAngle() {
        return inputs.hoodAngleDegrees;
    }
    
    // ==================== Flywheel Control ====================
    
    /**
     * Set flywheel voltage (percentage of max)
     * @param percent Power percentage (-1.0 to 1.0)
     */
    public void setFlywheelPercent(double percent) {
        double clampedPercent = MathUtil.clamp(percent, -1.0, 1.0);
        io.setFlywheelVoltage(clampedPercent * MAX_VOLTAGE);
    }
    
    /**
     * Set flywheel voltage directly
     * @param voltage Voltage to apply (-12 to 12)
     */
    public void setFlywheelVoltage(double voltage) {
        io.setFlywheelVoltage(voltage);
    }
    
    /**
     * Set flywheel target velocity
     * @param rpm Target velocity in RPM
     */
    public void setFlywheelVelocity(double rpm) {
        io.setFlywheelVelocity(rpm);
    }
    
    /**
     * Get the current flywheel velocity
     * @return Flywheel velocity in RPM
     */
    public double getFlywheelVelocityRPM() {
        return inputs.flywheelVelocityRPM;
    }
    
    /**
     * Get the current flywheel target velocity
     * @return Flywheel target velocity in RPM
     */
    public double getFlywheelTargetRPM() {
        return inputs.flywheelTargetRPM;
    }
    
    /**
     * Check if flywheel is at target speed
     * @param targetRPM Target RPM
     * @param toleranceRPM Acceptable tolerance
     * @return True if within tolerance
     */
    public boolean isFlywheelAtSpeed(double targetRPM, double toleranceRPM) {
        return Math.abs(inputs.flywheelVelocityRPM - targetRPM) < toleranceRPM;
    }
    
    /**
     * Check if flywheel is at its current target speed
     * @param toleranceRPM Acceptable tolerance
     * @return True if within tolerance of target
     */
    public boolean isFlywheelAtTargetSpeed(double toleranceRPM) {
        return Math.abs(inputs.flywheelVelocityRPM - inputs.flywheelTargetRPM) < toleranceRPM;
    }
    
    // ==================== Combined Control ====================
    
    /**
     * Aim the turret at a specific position
     * @param turretAngleDegrees Turret angle
     * @param hoodAngleDegrees Hood angle
     */
    public void aim(double turretAngleDegrees, double hoodAngleDegrees) {
        setTurretAngle(turretAngleDegrees);
        setHoodAngle(hoodAngleDegrees);
    }
    
    /**
     * Prepare to shoot with specific parameters (using voltage)
     * @param turretAngleDegrees Turret angle
     * @param hoodAngleDegrees Hood angle
     * @param flywheelVoltage Flywheel voltage
     */
    public void prepareShot(double turretAngleDegrees, double hoodAngleDegrees, double flywheelVoltage) {
        aim(turretAngleDegrees, hoodAngleDegrees);
        setFlywheelVoltage(flywheelVoltage);
    }
    
    /**
     * Prepare to shoot with specific parameters (using velocity control)
     * @param turretAngleDegrees Turret angle
     * @param hoodAngleDegrees Hood angle
     * @param flywheelRPM Flywheel target velocity in RPM
     */
    public void prepareShotVelocity(double turretAngleDegrees, double hoodAngleDegrees, double flywheelRPM) {
        aim(turretAngleDegrees, hoodAngleDegrees);
        setFlywheelVelocity(flywheelRPM);
    }
    
    /**
     * Stop all turret motors
     */
    public void stop() {
        io.stop();
    }
    
    /**
     * Stop only the flywheel
     */
    public void stopFlywheel() {
        io.setFlywheelVoltage(0.0);
    }
}
