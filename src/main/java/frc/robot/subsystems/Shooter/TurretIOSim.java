package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ShooterConstants;

public class TurretIOSim implements TurretIO {
    private final DCMotorSim turretMotor;
    private final DCMotorSim flywheelMotor;

    // PID controller for turret position control
    private final PIDController turretPID;
    
    // PID + FF controller for flywheel velocity control
    private final PIDController flywheelPID;
    private final SimpleMotorFeedforward flywheelFF;

    // Robot constants for simulation
    private static final double TURRET_GEAR_RATIO = 50.0; // 50:1 gear ratio
    private static final double FLYWHEEL_GEAR_RATIO = 3.0; // Direct drive
    
    private static final double TURRET_MOI = 0.01; // Moment of inertia (kg*m^2)
    private static final double FLYWHEEL_MOI = 0.001;

    // Applied voltages for logging
    private double turretAppliedVolts = 0.0;
    private double flywheelAppliedVolts = 0.0;
    
    // Position control setpoint (null = voltage control mode)
    private Double turretSetpointDegrees = null;
    
    // Velocity control setpoint (null = voltage control mode)
    private Double flywheelSetpointRPM = null;
    
    // Servo simulation - servos move to position over time
    private double hoodCurrentAngleDegrees = 0.0;
    private double hoodTargetAngleDegrees = 0.0;
    private static final double SERVO_SPEED_DEG_PER_SEC = 300.0; // Typical servo speed
    
    // Simulation loop time
    private static final double LOOP_PERIOD_SECS = 0.02; // 20ms

    public TurretIOSim() {
        // Initialize motor simulators with NEO motor model
        turretMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), TURRET_MOI, TURRET_GEAR_RATIO),
            DCMotor.getNEO(1)
        );
        flywheelMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), FLYWHEEL_MOI, FLYWHEEL_GEAR_RATIO),
            DCMotor.getNeo550(1)
        );
        
        // Initialize PID controller for turret position control
        turretPID = new PIDController(0.1, 0.0, 0.01);
        turretPID.setTolerance(1.0); // 1 degree tolerance
        
        // Initialize PID + FF for flywheel velocity control
        flywheelPID = new PIDController(
            ShooterConstants.kFlywheelP, 
            ShooterConstants.kFlywheelI, 
            ShooterConstants.kFlywheelD
        );
        flywheelFF = new SimpleMotorFeedforward(0.0, ShooterConstants.kFlywheelFF, 0.0);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // Run turret position control if setpoint is active
        if (turretSetpointDegrees != null) {
            double currentAngle = getTurretAngleDegrees();
            double output = turretPID.calculate(currentAngle, turretSetpointDegrees);
            turretAppliedVolts = MathUtil.clamp(output, -12.0, 12.0);
            turretMotor.setInputVoltage(turretAppliedVolts);
        }
        
        // Run flywheel velocity control if setpoint is active
        if (flywheelSetpointRPM != null) {
            double currentRPM = getFlywheelRPM();
            double pidOutput = flywheelPID.calculate(currentRPM, flywheelSetpointRPM);
            double ffOutput = flywheelFF.calculate(flywheelSetpointRPM);
            flywheelAppliedVolts = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
            flywheelMotor.setInputVoltage(flywheelAppliedVolts);
        }
        
        // Simulate servo movement toward target position
        updateServoSimulation();
        
        // Update motor simulations
        turretMotor.update(LOOP_PERIOD_SECS);
        flywheelMotor.update(LOOP_PERIOD_SECS);

        // Set turret inputs
        inputs.turretVoltage = turretAppliedVolts;
        inputs.turretCurrent = turretMotor.getCurrentDrawAmps();
        inputs.turretAngleDegrees = getTurretAngleDegrees();
        
        // Set hood (servo) inputs - servos don't have voltage/current in the same way
        // inputs.hoodVoltage = 0.0; // Servos don't report voltage
        // inputs.hoodCurrent = 0.0; // Servos don't report current
        inputs.hoodAngleDegrees = hoodCurrentAngleDegrees;
        
        // Set flywheel inputs
        inputs.flywheelVoltage = flywheelAppliedVolts;
        inputs.flywheelCurrent = flywheelMotor.getCurrentDrawAmps();
        inputs.flywheelVelocityRPM = getFlywheelRPM();
        inputs.flywheelTargetRPM = flywheelSetpointRPM != null ? flywheelSetpointRPM : 0.0;
    }
    
    /** Simulate servo moving toward target position */
    private void updateServoSimulation() {
        double error = hoodTargetAngleDegrees - hoodCurrentAngleDegrees;
        double maxMovement = SERVO_SPEED_DEG_PER_SEC * LOOP_PERIOD_SECS;
        
        if (Math.abs(error) <= maxMovement) {
            // Close enough, snap to target
            hoodCurrentAngleDegrees = hoodTargetAngleDegrees;
        } else {
            // Move toward target at servo speed
            hoodCurrentAngleDegrees += Math.copySign(maxMovement, error);
        }
    }
    
    /** Get turret angle in degrees from simulation */
    private double getTurretAngleDegrees() {
        return Units.radiansToDegrees(turretMotor.getAngularPositionRad());
    }
    
    /** Get flywheel velocity in RPM from simulation */
    private double getFlywheelRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(
            flywheelMotor.getAngularVelocityRadPerSec());
    }

    @Override
    public void setTurretVoltage(double voltage) {
        // Disable position control, switch to voltage control
        turretSetpointDegrees = null;
        turretAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        turretMotor.setInputVoltage(turretAppliedVolts);
    }

    // @Override
    // public void setHoodVoltage(double voltage) {
    //     // Servos don't use voltage control - this is a no-op
    //     // Use setHoodAngle() instead
    // }

    @Override
    public void setFlywheelVoltage(double voltage) {
        // Disable velocity control, switch to voltage control
        flywheelSetpointRPM = null;
        flywheelAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        flywheelMotor.setInputVoltage(flywheelAppliedVolts);
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        // Enable velocity control mode
        flywheelSetpointRPM = rpm;
    }

    @Override
    public void setTurretAngle(double angleDegrees) {
        // Enable position control mode
        turretSetpointDegrees = angleDegrees;
    }

    // @Override
    // public void setHoodAngle(double angleDegrees) {
    //     // Set servo target position (clamped to valid range)
    //     hoodTargetAngleDegrees = MathUtil.clamp(angleDegrees, 
    //         ShooterConstants.kHoodMinAngle, 
    //         ShooterConstants.kHoodMaxAngle);
    // }

    @Override
    public void stop() {
        turretSetpointDegrees = null;
        flywheelSetpointRPM = null;
        setTurretVoltage(0.0);
        setFlywheelVoltage(0.0);
        // Servo holds position when stopped (no action needed)
    }
}
