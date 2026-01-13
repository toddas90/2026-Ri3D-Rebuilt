package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

/**
 * Command that toggles intake between deployed/running and stowed/stopped states
 */
public class IntakeToggleCommand extends Command {
    private final Intake intake;
    private boolean intakeDeployed = false;
    
    public IntakeToggleCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        if (!intakeDeployed) {
            // Deploy intake and start rollers
            intake.setArmPosition(Intake.ArmPosition.OUT);
            intake.startRoller();
            intakeDeployed = true;
        } else {
            // Stop rollers and stow intake
            intake.stopRoller();
            intake.setArmPosition(Intake.ArmPosition.IN);
            intakeDeployed = false;
        }
    }
    
    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
    
    /**
     * Check if intake is currently deployed
     */
    public boolean isIntakeDeployed() {
        return intakeDeployed;
    }
}