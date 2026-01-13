package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private static final double ROLLER_VOLTAGE = IntakeConstants.kIntakeVoltage;

    public enum ArmPosition {
        IN(0.0),
        OUT(90.0); //probably a different degree, need to test
        
        public final double angleDegrees;
        
        ArmPosition(double angleDegrees) {
            this.angleDegrees = angleDegrees;
        }
    }

    public Intake(IntakeIO io) {
        this.io = io;

        //Start with brake mode enabled for safety
        io.setBrakeMode(true);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Log additional data
        Logger.recordOutput("Intake/ArmPositionDegrees", inputs.armPositionDegrees);
        Logger.recordOutput("Intake/ArmSetpointDegrees", inputs.armSetPointDegrees);
        Logger.recordOutput("Intake/ArmCurrentAmps", inputs.armAppliedCurrentAmps);
        Logger.recordOutput("Intake/ArmAppliedVolts", inputs.armAppliedVolts);
        Logger.recordOutput("Intake/RollerCurrentAmps", inputs.rollerAppliedCurrentAmps);
        Logger.recordOutput("Intake/RollerAppliedVolts", inputs.rollerAppliedVolts);

    }

    // ==================== Roller Control ====================

    /**
     * Start roller intake
     */
    public void startRoller() {
        io.setRollerVoltage(ROLLER_VOLTAGE);
    }

    /**
     * Stop roller
     */
    public void stopRoller() {
        io.setRollerVoltage(0);
    }

    public boolean isRollerRunning() {
        return inputs.rollerAppliedCurrentAmps > 0.1;
    }

    // ==================== Arm Control ====================

    /**
     * Set arm to a preset position
     * @param position The preset position
     */
    public void setArmPosition(ArmPosition position) {
        io.setArmPosition(position.angleDegrees);
    }

    /**
     * Set arm to a specific angle
     * @param angleDegrees Target angle in degrees
     */
    public void setArmAngle(double angleDegrees) {
        double clamped = Math.max(IntakeConstants.kInPosition, Math.min(IntakeConstants.kOutPosition, angleDegrees)); //clamp between 0 and 90 degrees
        io.setArmPosition(clamped);
    }

    /**
     * Get current arm angle
     * @return Arm angle in degrees
     */
    public double getArmAngle() {
        return inputs.armPositionDegrees;
    }

    /**
     * Check if arm is at target position
     * @param position Target position
     */
    public boolean isArmAtPosition(ArmPosition position) {
        return Math.abs(inputs.armPositionDegrees - position.angleDegrees) < 
               IntakeConstants.kArmPositionTolerance;
    }

    // ==================== General Control ====================

    /**
     * Stop all intake motors
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
     * Set whether brake mode is enabled
     * @param enabled True to enable brake mode
     */
    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

}
