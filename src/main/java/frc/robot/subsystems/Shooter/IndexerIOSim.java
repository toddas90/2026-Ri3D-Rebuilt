package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
    // Simulation Motor
    private final DCMotorSim indexerMotor;

    // Robot constants for simulation
    private static final double GEAR_RATIO = 10; // Ballpark
    private static final double MOI = 0.025; // Moment of inertia (kg*m^2)
    
    // Applied voltages for logging
    private double indexerAppliedVolts = 0.0;
    
    // Simulation loop time
    private static final double LOOP_PERIOD_SECS = 0.02; // 20ms

    public IndexerIOSim() {
        // Initialize motor simulators with CIM motor model
        indexerMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), MOI, GEAR_RATIO),
            DCMotor.getCIM(1)
        );
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Update simulation
        indexerMotor.update(LOOP_PERIOD_SECS);
        
        // Set voltage inputs
        inputs.indexerVoltage = indexerAppliedVolts;
        inputs.indexerCurrent = indexerMotor.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double voltage) {
        // Apply voltage to motor simulator
        indexerAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        indexerMotor.setInputVoltage(voltage);
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }
}
