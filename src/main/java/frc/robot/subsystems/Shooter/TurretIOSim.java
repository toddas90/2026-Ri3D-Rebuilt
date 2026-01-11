package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
    private final DCMotorSim turretMotor;
    private final DCMotorSim hoodMotor;
    private final DCMotorSim flywheelMotor;

    // PID controllers for position control
    private final PIDController turretPID;
    private final PIDController hoodPID;

    // Robot constants for simulation
    private static final double TURRET_GEAR_RATIO = 50.0; // 50:1 gear ratio
    private static final double HOOD_GEAR_RATIO = 25.0;   // 25:1 gear ratio
    private static final double FLYWHEEL_GEAR_RATIO = 1.0; // Direct drive
    
    private static final double TURRET_MOI = 0.01; // Moment of inertia (kg*m^2)
    private static final double HOOD_MOI = 0.005;
    private static final double FLYWHEEL_MOI = 0.001;

    // Applied voltages for logging
    private double turretAppliedVolts = 0.0;
    private double hoodAppliedVolts = 0.0;
    private double flywheelAppliedVolts = 0.0;
    
    // Position control setpoints (null = voltage control mode)
    private Double turretSetpointDegrees = null;
    private Double hoodSetpointDegrees = null;
    
    // Simulation loop time
    private static final double LOOP_PERIOD_SECS = 0.02; // 20ms

    public TurretIOSim() {
        // Initialize motor simulators with NEO motor model
        turretMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), TURRET_MOI, TURRET_GEAR_RATIO),
            DCMotor.getNEO(1)
        );
        hoodMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), HOOD_MOI, HOOD_GEAR_RATIO),
            DCMotor.getNEO(1)
        );
        flywheelMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), FLYWHEEL_MOI, FLYWHEEL_GEAR_RATIO),
            DCMotor.getNEO(1)
        );
        
        // Initialize PID controllers for position control
        turretPID = new PIDController(0.1, 0.0, 0.01);
        turretPID.setTolerance(1.0); // 1 degree tolerance
        
        hoodPID = new PIDController(0.15, 0.0, 0.01);
        hoodPID.setTolerance(0.5); // 0.5 degree tolerance
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // Run position control if setpoints are active
        if (turretSetpointDegrees != null) {
            double currentAngle = getTurretAngleDegrees();
            double output = turretPID.calculate(currentAngle, turretSetpointDegrees);
            turretAppliedVolts = MathUtil.clamp(output, -12.0, 12.0);
            turretMotor.setInputVoltage(turretAppliedVolts);
        }
        
        if (hoodSetpointDegrees != null) {
            double currentAngle = getHoodAngleDegrees();
            double output = hoodPID.calculate(currentAngle, hoodSetpointDegrees);
            hoodAppliedVolts = MathUtil.clamp(output, -12.0, 12.0);
            hoodMotor.setInputVoltage(hoodAppliedVolts);
        }
        
        // Update simulation
        turretMotor.update(LOOP_PERIOD_SECS);
        hoodMotor.update(LOOP_PERIOD_SECS);
        flywheelMotor.update(LOOP_PERIOD_SECS);

        // Set voltage inputs
        inputs.turretVoltage = turretAppliedVolts;
        inputs.turretCurrent = turretMotor.getCurrentDrawAmps();
        inputs.turretAngleDegrees = getTurretAngleDegrees();
        
        inputs.hoodVoltage = hoodAppliedVolts;
        inputs.hoodCurrent = hoodMotor.getCurrentDrawAmps();
        inputs.hoodAngleDegrees = getHoodAngleDegrees();
        
        inputs.flywheelVoltage = flywheelAppliedVolts;
        inputs.flywheelCurrent = flywheelMotor.getCurrentDrawAmps();
    }
    
    /** Get turret angle in degrees from simulation */
    private double getTurretAngleDegrees() {
        return Units.radiansToDegrees(turretMotor.getAngularPositionRad());
    }
    
    /** Get hood angle in degrees from simulation */
    private double getHoodAngleDegrees() {
        return Units.radiansToDegrees(hoodMotor.getAngularPositionRad());
    }

    @Override
    public void setTurretVoltage(double voltage) {
        // Disable position control, switch to voltage control
        turretSetpointDegrees = null;
        turretAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        turretMotor.setInputVoltage(turretAppliedVolts);
    }

    @Override
    public void setHoodVoltage(double voltage) {
        // Disable position control, switch to voltage control
        hoodSetpointDegrees = null;
        hoodAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        hoodMotor.setInputVoltage(hoodAppliedVolts);
    }

    @Override
    public void setFlywheelVoltage(double voltage) {
        flywheelAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        flywheelMotor.setInputVoltage(flywheelAppliedVolts);
    }

    @Override
    public void setTurretAngle(double angleDegrees) {
        // Enable position control mode
        turretSetpointDegrees = angleDegrees;
    }

    @Override
    public void setHoodAngle(double angleDegrees) {
        // Enable position control mode
        hoodSetpointDegrees = angleDegrees;
    }

    @Override
    public void stop() {
        turretSetpointDegrees = null;
        hoodSetpointDegrees = null;
        setTurretVoltage(0.0);
        setHoodVoltage(0.0);
        setFlywheelVoltage(0.0);
    }
}
