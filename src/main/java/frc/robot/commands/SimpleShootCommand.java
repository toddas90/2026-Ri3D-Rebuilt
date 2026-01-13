package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.FixedShooter;
import frc.robot.Constants.ShooterConstants;

/**
 * Command that spins up flywheels to a preset speed without aiming
 */
public class SimpleShootCommand extends Command {
    private final FixedShooter shooter;
    private final double targetRPM;
    
    // Default RPM for close-range shooting
    private static final double DEFAULT_RPM = ShooterConstants.kMaxFlywheelRPM;
    
    public SimpleShootCommand(FixedShooter shooter) {
        this(shooter, DEFAULT_RPM);
    }
    
    public SimpleShootCommand(FixedShooter shooter, double rpm) {
        this.shooter = shooter;
        this.targetRPM = Math.min(rpm, ShooterConstants.kMaxFlywheelRPM);
        
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        // Spin up flywheels to target speed
        shooter.setFlywheelVelocity(targetRPM);
    }
    
    @Override
    public void execute() {
        // Keep flywheels at speed
        shooter.setFlywheelVelocity(targetRPM);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop flywheels when command ends
        shooter.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
    
    /**
     * Check if flywheels are at speed
     */
    public boolean isReadyToShoot() {
        return shooter.areFlywheelsAtSpeed(100); // 100 RPM tolerance
    }
}