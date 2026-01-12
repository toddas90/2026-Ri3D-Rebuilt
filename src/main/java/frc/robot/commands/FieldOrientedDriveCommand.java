package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.Drive;

import java.util.function.DoubleSupplier;

/**
 * Command for field-oriented driving with joystick inputs.
 */
public class FieldOrientedDriveCommand extends Command {
    private final Drive drive;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final DoubleSupplier rotationSupplier;
    private final double deadband;
    private final boolean squareInputs;

    /**
     * Creates a field-oriented drive command.
     * 
     * @param drive The drive subsystem
     * @param xSpeedSupplier Supplier for forward/backward speed (-1 to 1)
     * @param ySpeedSupplier Supplier for left/right speed (-1 to 1)
     * @param rotationSupplier Supplier for rotation speed (-1 to 1)
     * @param deadband Deadband to apply to inputs
     */
    public FieldOrientedDriveCommand(
            Drive drive,
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier rotationSupplier,
            double deadband) {
        this(drive, xSpeedSupplier, ySpeedSupplier, rotationSupplier, deadband, true);
    }

    /**
     * Creates a field-oriented drive command.
     * 
     * @param drive The drive subsystem
     * @param xSpeedSupplier Supplier for forward/backward speed (-1 to 1)
     * @param ySpeedSupplier Supplier for left/right speed (-1 to 1)
     * @param rotationSupplier Supplier for rotation speed (-1 to 1)
     * @param deadband Deadband to apply to inputs
     * @param squareInputs Whether to square inputs for finer control
     */
    public FieldOrientedDriveCommand(
            Drive drive,
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier rotationSupplier,
            double deadband,
            boolean squareInputs) {
        this.drive = drive;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotationSupplier = rotationSupplier;
        this.deadband = deadband;
        this.squareInputs = squareInputs;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Get joystick inputs
        double xSpeed = xSpeedSupplier.getAsDouble();
        double ySpeed = ySpeedSupplier.getAsDouble();
        double rotation = rotationSupplier.getAsDouble();

        // Apply deadband
        xSpeed = MathUtil.applyDeadband(xSpeed, deadband);
        ySpeed = MathUtil.applyDeadband(ySpeed, deadband);
        rotation = MathUtil.applyDeadband(rotation, deadband);

        // Square inputs for finer control (while preserving sign)
        if (squareInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
            rotation = Math.copySign(rotation * rotation, rotation);
        }

        // Drive field-oriented
        drive.driveFieldOriented(xSpeed, ySpeed, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Default command, runs continuously
    }
}
