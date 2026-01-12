package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Indexer;
import frc.robot.subsystems.Shooter.Turret;

/**
 * Command for manual flywheel control at full voltage.
 */
public class ManualShootCommand extends Command {
    private final Turret leftTurret;
    private final Turret rightTurret;
    private final Indexer indexer;
    private final double voltage;

    /**
     * Creates a manual shoot command at full voltage (12V).
     * 
     * @param leftTurret The left turret subsystem
     * @param rightTurret The right turret subsystem
     * @param indexer The indexer subsystem
     */
    public ManualShootCommand(Turret leftTurret, Turret rightTurret, Indexer indexer) {
        this(leftTurret, rightTurret, indexer, 12.0);
    }

    /**
     * Creates a manual shoot command at specified voltage.
     * 
     * @param leftTurret The left turret subsystem
     * @param rightTurret The right turret subsystem
     * @param indexer The indexer subsystem
     * @param voltage The voltage to apply to flywheels
     */
    public ManualShootCommand(Turret leftTurret, Turret rightTurret, Indexer indexer, double voltage) {
        this.leftTurret = leftTurret;
        this.rightTurret = rightTurret;
        this.indexer = indexer;
        this.voltage = voltage;

        addRequirements(leftTurret, rightTurret, indexer);
    }

    @Override
    public void execute() {
        indexer.start();
        leftTurret.setFlywheelVoltage(voltage);
        rightTurret.setFlywheelVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        leftTurret.stopFlywheel();
        rightTurret.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
