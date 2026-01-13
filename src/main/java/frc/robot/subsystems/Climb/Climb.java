package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    
    // Preset positions
    public enum LiftPosition {
        STOWED(ClimbConstants.kLiftMinHeight),
        BAR_INSERT(ClimbConstants.kLiftBarInsertHeight),
        EXTENDED(ClimbConstants.kLiftMaxHeight);
        
        public final double heightMeters;
        
        LiftPosition(double heightMeters) {
            this.heightMeters = heightMeters;
        }
    }
    
    public enum PivotPosition {
        NORMAL(0.0),
        FLIPPED(180.0);
        
        public final double angleDegrees;
        
        PivotPosition(double angleDegrees) {
            this.angleDegrees = angleDegrees;
        }
    }
    
    public Climb(ClimbIO io) {
        this.io = io;
        
        // Start with brake mode enabled for safety
        io.setBrakeMode(true);
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
        
        // Log additional data
        Logger.recordOutput("Climb/LiftPositionMeters", inputs.liftPositionMeters);
        Logger.recordOutput("Climb/LiftSetpointMeters", inputs.liftSetpointMeters);
        Logger.recordOutput("Climb/LiftVelocityMPS", inputs.liftVelocityMetersPerSec);
        Logger.recordOutput("Climb/PivotPositionDegrees", inputs.pivotPositionDegrees);
        Logger.recordOutput("Climb/PivotVelocityDPS", inputs.pivotVelocityDegreesPerSec);
        Logger.recordOutput("Climb/LiftCurrent", inputs.liftCurrentAmps);
        Logger.recordOutput("Climb/PivotCurrent", inputs.pivotCurrentAmps);
    }
    
    // ==================== Lift Control ====================
    
    /**
     * Set lift to a preset position
     * @param position The preset position
     */
    public void setLiftPosition(LiftPosition position) {
        io.setLiftPosition(position.heightMeters);
    }
    
    /**
     * Set lift to a specific height
     * @param heightMeters Target height in meters
     */
    public void setLiftHeight(double heightMeters) {
        double clamped = MathUtil.clamp(
            heightMeters,
            ClimbConstants.kLiftMinHeight,
            ClimbConstants.kLiftMaxHeight
        );
        io.setLiftPosition(clamped);
    }
    
    /**
     * Check if the lift is under load (robot is hanging)
     * @return true if lift current indicates robot weight
     */
    public boolean isHooked() {
        // When lifting the robot, current will be significantly higher
        return inputs.liftCurrentAmps > ClimbConstants.kLiftHookedCurrentThreshold;
    }
    
    /**
     * Set lift to a specific height with smart power management
     * @param heightMeters Target height in meters
     */
    public void setLiftHeightSmart(double heightMeters) {
        double clamped = MathUtil.clamp(
            heightMeters,
            ClimbConstants.kLiftMinHeight,
            ClimbConstants.kLiftMaxHeight
        );
        
        // If going down and not hooked, use reduced power
        if (heightMeters < inputs.liftPositionMeters && !isHooked()) {
            // Override with voltage control for gentle descent
            io.setLiftVoltage(-4.8); // 40% of 12V
        } else {
            // Use normal position control (will use configured power limits)
            io.setLiftPosition(clamped);
        }
    }

    public void setLiftPositionSmart(LiftPosition position) {
        setLiftHeightSmart(position.heightMeters);
    }
    
    /**
     * Get current lift height
     * @return Height in meters
     */
    public double getLiftHeight() {
        return inputs.liftPositionMeters;
    }
    
    /**
     * Check if lift is at target position
     * @param position Target position
     * @return True if within tolerance
     */
    public boolean isLiftAtPosition(LiftPosition position) {
        return Math.abs(inputs.liftPositionMeters - position.heightMeters) < 
               ClimbConstants.kLiftPositionTolerance;
    }
    
    // ==================== Pivot Control ====================
    
    /**
     * Set pivot to a preset position
     * @param position The preset position
     */
    public void setPivotPosition(PivotPosition position) {
        io.setPivotAngle(position.angleDegrees);
    }
    
    /**
     * Set pivot to a specific angle
     * @param angleDegrees Target angle in degrees
     */
    public void setPivotAngle(double angleDegrees) {
        double clamped = MathUtil.clamp(
            angleDegrees,
            ClimbConstants.kPivotMinAngle,
            ClimbConstants.kPivotMaxAngle
        );
        io.setPivotAngle(clamped);
    }
    
    /**
     * Manually control pivot with voltage
     * @param voltage Voltage to apply (-12 to 12)
     */
    public void setPivotVoltage(double voltage) {
        io.setPivotVoltage(voltage);
    }
    
    /**
     * Get current pivot angle
     * @return Angle in degrees
     */
    public double getPivotAngle() {
        return inputs.pivotPositionDegrees;
    }
    
    /**
     * Check if pivot is at target position
     * @param position Target position
     * @return True if within tolerance
     */
    public boolean isPivotAtPosition(PivotPosition position) {
        return Math.abs(inputs.pivotPositionDegrees - position.angleDegrees) < 
               ClimbConstants.kPivotPositionTolerance;
    }
    
    // ==================== General Control ====================
    
    /**
     * Stop all climb motors
     */
    public void stop() {
        io.stop();
    }
    
    /**
     * Reset encoders to zero (use with caution)
     */
    public void resetEncoders() {
        io.resetEncoders();
    }
    
    /**
     * Set brake mode for motors
     * @param enabled True to enable brake mode
     */
    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }
    
    /**
     * Check if climb is safe to operate
     * @return True if temperatures and currents are within limits
     */
    public boolean isSafeToOperate() {
        boolean liftSafe = inputs.liftTempCelsius < ClimbConstants.kMaxMotorTemp &&
                           inputs.liftCurrentAmps < ClimbConstants.kLiftCurrentLimit;
        boolean pivotSafe = inputs.pivotTempCelsius < ClimbConstants.kMaxMotorTemp &&
                            inputs.pivotCurrentAmps < ClimbConstants.kPivotCurrentLimit;
        return liftSafe && pivotSafe;
    }
}
