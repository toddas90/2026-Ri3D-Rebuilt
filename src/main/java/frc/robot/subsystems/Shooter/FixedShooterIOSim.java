package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ShooterConstants;

public class FixedShooterIOSim implements FixedShooterIO {
    // Simulated flywheel motors
    private final DCMotorSim leftFlywheelSim;
    private final DCMotorSim rightFlywheelSim;
    
    // PID controllers for velocity control
    private final PIDController leftPID;
    private final PIDController rightPID;
    
    // Control mode tracking
    private boolean leftVelocityMode = false;
    private boolean rightVelocityMode = false;
    
    // Applied voltages and setpoints
    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double leftTargetRPM = 0.0;
    private double rightTargetRPM = 0.0;
    
    // Flywheel moment of inertia (kg*m^2)
    private static final double FLYWHEEL_MOI = 0.003; // Approximate for a small flywheel
    
    // Simulation loop time
    private static final double LOOP_PERIOD_SECS = 0.02; // 20ms
    
    public FixedShooterIOSim() {
        // Initialize flywheel simulators with NEO 550 motors
        leftFlywheelSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1), 
                FLYWHEEL_MOI, 
                ShooterConstants.kFlywheelGearRatio
            ),
            DCMotor.getNeo550(1)
        );
        
        rightFlywheelSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1), 
                FLYWHEEL_MOI, 
                ShooterConstants.kFlywheelGearRatio
            ),
            DCMotor.getNeo550(1)
        );
        
        // Initialize PID controllers
        leftPID = new PIDController(
            ShooterConstants.kFlywheelP,
            ShooterConstants.kFlywheelI,
            ShooterConstants.kFlywheelD
        );
        
        rightPID = new PIDController(
            ShooterConstants.kFlywheelP,
            ShooterConstants.kFlywheelI,
            ShooterConstants.kFlywheelD
        );
    }
    
    @Override
    public void updateInputs(FixedShooterIOInputs inputs) {
        // Run PID control if in velocity mode
        if (leftVelocityMode) {
            double currentRPM = getMotorRPM(leftFlywheelSim);
            double pidOutput = leftPID.calculate(currentRPM, leftTargetRPM);
            double ffOutput = ShooterConstants.kFlywheelFF * leftTargetRPM;
            leftAppliedVolts = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
            leftFlywheelSim.setInputVoltage(leftAppliedVolts);
        }
        
        if (rightVelocityMode) {
            double currentRPM = getMotorRPM(rightFlywheelSim);
            double pidOutput = rightPID.calculate(currentRPM, rightTargetRPM);
            double ffOutput = ShooterConstants.kFlywheelFF * rightTargetRPM;
            rightAppliedVolts = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
            rightFlywheelSim.setInputVoltage(rightAppliedVolts);
        }
        
        // Update simulations
        leftFlywheelSim.update(LOOP_PERIOD_SECS);
        rightFlywheelSim.update(LOOP_PERIOD_SECS);
        
        // Set inputs
        inputs.leftFlywheelVoltage = leftAppliedVolts;
        inputs.leftFlywheelCurrent = leftFlywheelSim.getCurrentDrawAmps();
        inputs.leftFlywheelVelocityRPM = getMotorRPM(leftFlywheelSim);
        inputs.leftFlywheelTargetRPM = leftTargetRPM;
        
        inputs.rightFlywheelVoltage = rightAppliedVolts;
        inputs.rightFlywheelCurrent = rightFlywheelSim.getCurrentDrawAmps();
        inputs.rightFlywheelVelocityRPM = getMotorRPM(rightFlywheelSim);
        inputs.rightFlywheelTargetRPM = rightTargetRPM;
    }

    @Override
    public void setLeftFlywheelVoltage(double voltage) {
        leftVelocityMode = false;
        leftTargetRPM = 0;
        leftAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        leftFlywheelSim.setInputVoltage(leftAppliedVolts);
    }

    @Override
    public void setRightFlywheelVoltage(double voltage) {
        rightVelocityMode = false;
        rightTargetRPM = 0;
        rightAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        rightFlywheelSim.setInputVoltage(rightAppliedVolts);
    }

    @Override
    public void setLeftFlywheelVelocity(double rpm) {
        leftVelocityMode = true;
        leftTargetRPM = rpm;
    }

    @Override
    public void setRightFlywheelVelocity(double rpm) {
        rightVelocityMode = true;
        rightTargetRPM = rpm;
    }

    @Override
    public void stop() {
        leftVelocityMode = false;
        rightVelocityMode = false;
        leftTargetRPM = 0;
        rightTargetRPM = 0;
        setLeftFlywheelVoltage(0);
        setRightFlywheelVoltage(0);
    }
    
    /**
     * Convert motor angular velocity to RPM accounting for gear ratio
     */
    private double getMotorRPM(DCMotorSim motor) {
        // Convert from rad/s to RPM and account for gear ratio
        double motorRadPerSec = motor.getAngularVelocityRadPerSec();
        double motorRPM = motorRadPerSec * 60.0 / (2 * Math.PI);
        return motorRPM / ShooterConstants.kFlywheelGearRatio;
    }
}
