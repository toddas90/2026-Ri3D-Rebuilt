package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive.Drive;

public class DriveIntakeForwardCommand extends Command {
    private final Drive drive;
    private final java.util.function.DoubleSupplier xSpeedSupplier;
    private final java.util.function.DoubleSupplier ySpeedSupplier;
    
    public DriveIntakeForwardCommand(Drive drive, 
                                     java.util.function.DoubleSupplier xSpeedSupplier,
                                     java.util.function.DoubleSupplier ySpeedSupplier) {
        this.drive = drive;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        double xSpeed = xSpeedSupplier.getAsDouble();
        double ySpeed = ySpeedSupplier.getAsDouble();
        
        // Calculate desired angle from joystick inputs
        double desiredAngle = Math.atan2(ySpeed, xSpeed);
        
        // Get current robot heading
        double currentAngle = drive.getHeading().getRadians();
        
        // Calculate angle error
        double angleError = desiredAngle - currentAngle;
        
        // Normalize angle error to [-pi, pi]
        angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError));
        
        // Simple P controller for rotation
        double kP = DriveConstants.kIntakeForwardP;
        double rotation = kP * angleError;
        
        // Clamp rotation to [-1, 1]
        rotation = MathUtil.clamp(rotation, -1.0, 1.0);
        
        // Drive field-oriented with calculated rotation
        drive.driveFieldOriented(xSpeed, ySpeed, rotation);
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
