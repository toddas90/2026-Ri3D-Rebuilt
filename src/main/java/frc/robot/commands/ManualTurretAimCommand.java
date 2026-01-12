package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Turret;

import java.util.function.DoubleSupplier;

/**
 * Command for manually aiming a turret using joystick input.
 * The turret angle is set based on the joystick direction.
 */
public class ManualTurretAimCommand extends Command {
    private final Turret turret;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final double deadband;

    /**
     * Creates a manual turret aim command.
     * 
     * @param turret The turret subsystem to control
     * @param xSupplier Supplier for joystick X axis
     * @param ySupplier Supplier for joystick Y axis
     * @param deadband Minimum joystick magnitude to update angle
     */
    public ManualTurretAimCommand(
            Turret turret,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            double deadband) {
        this.turret = turret;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.deadband = deadband;

        addRequirements(turret);
    }

    /**
     * Creates a manual turret aim command with default deadband of 0.5.
     */
    public ManualTurretAimCommand(
            Turret turret,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
        this(turret, xSupplier, ySupplier, 0.5);
    }

    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        double y = ySupplier.getAsDouble();

        // Calculate magnitude
        double magnitude = Math.sqrt(x * x + y * y);

        // Only update angle if joystick is pushed past deadband
        if (magnitude > deadband) {
            // Calculate angle from joystick position
            // atan2(y, x) gives angle in radians, convert to degrees
            double angle = Math.toDegrees(Math.atan2(y, x));
            turret.setTurretAngle(angle);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
