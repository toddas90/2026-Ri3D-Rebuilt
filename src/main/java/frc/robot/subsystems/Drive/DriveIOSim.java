package frc.robot.subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class DriveIOSim implements DriveIO {
    // Create motor simulators for each wheel
    // Using CIM motors as specified in the hardware
    private final DCMotorSim frontLeftSim;
    private final DCMotorSim frontRightSim;
    private final DCMotorSim rearLeftSim;
    private final DCMotorSim rearRightSim;
    
    // Robot constants for simulation
    private static final double GEAR_RATIO = 8.45; // Typical for Toughbox Mini
    private static final double WHEEL_RADIUS_METERS = 0.0762; // 6 inch wheels
    private static final double MOI = 0.025; // Moment of inertia (kg*m^2) - estimated
    
    // Applied voltages for logging
    private double frontLeftAppliedVolts = 0.0;
    private double frontRightAppliedVolts = 0.0;
    private double rearLeftAppliedVolts = 0.0;
    private double rearRightAppliedVolts = 0.0;
    
    // Simulation loop time
    private static final double LOOP_PERIOD_SECS = 0.02; // 20ms
    
    public DriveIOSim() {
        // Initialize motor simulators with CIM motor model
        // Using LinearSystemId to create the motor plant
        frontLeftSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), MOI, GEAR_RATIO),
            DCMotor.getCIM(1)
        );
        
        frontRightSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), MOI, GEAR_RATIO),
            DCMotor.getCIM(1)
        );
        
        rearLeftSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), MOI, GEAR_RATIO),
            DCMotor.getCIM(1)
        );
        
        rearRightSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), MOI, GEAR_RATIO),
            DCMotor.getCIM(1)
        );
    }
    
    @Override
    public void updateInputs(DriveIOInputs inputs) {
        // Update simulation
        frontLeftSim.update(LOOP_PERIOD_SECS);
        frontRightSim.update(LOOP_PERIOD_SECS);
        rearLeftSim.update(LOOP_PERIOD_SECS);
        rearRightSim.update(LOOP_PERIOD_SECS);
        
        // Set voltage inputs
        inputs.frontLeftVoltage = frontLeftAppliedVolts;
        inputs.frontRightVoltage = frontRightAppliedVolts;
        inputs.rearLeftVoltage = rearLeftAppliedVolts;
        inputs.rearRightVoltage = rearRightAppliedVolts;
        
        // Set current draw from simulation
        inputs.frontLeftCurrent = frontLeftSim.getCurrentDrawAmps();
        inputs.frontRightCurrent = frontRightSim.getCurrentDrawAmps();
        inputs.rearLeftCurrent = rearLeftSim.getCurrentDrawAmps();
        inputs.rearRightCurrent = rearRightSim.getCurrentDrawAmps();
    }
    
    @Override
    public void setVoltage(double frontLeftVolts, double frontRightVolts, 
                           double rearLeftVolts, double rearRightVolts) {
        // Clamp voltages to battery voltage
        frontLeftAppliedVolts = MathUtil.clamp(frontLeftVolts, -12.0, 12.0);
        frontRightAppliedVolts = MathUtil.clamp(frontRightVolts, -12.0, 12.0);
        rearLeftAppliedVolts = MathUtil.clamp(rearLeftVolts, -12.0, 12.0);
        rearRightAppliedVolts = MathUtil.clamp(rearRightVolts, -12.0, 12.0);
        
        // Apply voltages to simulators
        frontLeftSim.setInputVoltage(frontLeftAppliedVolts);
        
        // Right side motors are inverted in the real robot
        frontRightSim.setInputVoltage(-frontRightAppliedVolts);
        
        rearLeftSim.setInputVoltage(rearLeftAppliedVolts);
        
        // Right side motors are inverted in the real robot
        rearRightSim.setInputVoltage(-rearRightAppliedVolts);
    }
    
    @Override
    public void stop() {
        setVoltage(0, 0, 0, 0);
    }
}
