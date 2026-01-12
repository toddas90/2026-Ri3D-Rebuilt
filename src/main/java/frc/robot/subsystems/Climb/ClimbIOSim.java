package frc.robot.subsystems.Climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOSim implements ClimbIO {
    // Simulated lift as an elevator
    private ElevatorSim liftSim;
    private final DCMotorSim pivotSim;
    
    // PID controllers for position control
    private final PIDController liftPID;
    private final PIDController pivotPID;
    
    // Control mode tracking
    private Double liftSetpointMeters = null;
    private Double pivotSetpointDegrees = null;
    
    // Applied voltages
    private double liftAppliedVolts = 0.0;
    private double pivotAppliedVolts = 0.0;
    
    // Simulation constants
    private static final double LOOP_PERIOD_SECS = 0.02; // 20ms
    private static final double LIFT_CARRIAGE_MASS_KG = 5.0; // Mass of just the lift carriage
    private static final double ROBOT_MASS_KG = 61.0; // ~135 lbs robot
    private static final double PIVOT_MOI = 0.1; // Moment of inertia for pivot
    
    // Track current pivot angle to determine load
    private double currentPivotAngleDegrees = 0.0;
    
    public ClimbIOSim() {
        // Initialize lift simulation with initial light load (just carriage)
        recreateLiftSim(LIFT_CARRIAGE_MASS_KG);
        
        // Initialize pivot simulation
        pivotSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1), 
                PIVOT_MOI, 
                ClimbConstants.kPivotGearRatio
            ),
            DCMotor.getNEO(1)
        );
        
        // Initialize PID controllers
        liftPID = new PIDController(
            ClimbConstants.kLiftP,
            ClimbConstants.kLiftI,
            ClimbConstants.kLiftD
        );
        liftPID.setTolerance(0.01); // 1cm tolerance
        
        pivotPID = new PIDController(
            ClimbConstants.kPivotP,
            ClimbConstants.kPivotI,
            ClimbConstants.kPivotD
        );
        pivotPID.setTolerance(1.0); // 1 degree tolerance
    }
    
    /**
     * Recreate the lift simulation with a new mass.
     * Preserves current position and velocity.
     */
    private void recreateLiftSim(double massKg) {
        double currentPosition = liftSim != null ? liftSim.getPositionMeters() : 0.0;
        double currentVelocity = liftSim != null ? liftSim.getVelocityMetersPerSecond() : 0.0;
        
        liftSim = new ElevatorSim(
            DCMotor.getNEO(1),
            ClimbConstants.kLiftGearRatio,
            massKg,
            ClimbConstants.kLiftDrumRadius,
            ClimbConstants.kLiftMinHeight,
            ClimbConstants.kLiftMaxHeight,
            true, // Simulate gravity
            currentPosition  // Starting position
        );
        
        // Restore velocity state
        liftSim.setState(currentPosition, currentVelocity);
    }
    
    /**
     * Calculate the effective mass on the lift based on pivot angle.
     * When upright (0°), lift only carries carriage.
     * When inverted (180°), lift carries entire robot.
     * Smooth transition between states.
     */
    private double calculateEffectiveLiftMass() {
        // Use cosine function for smooth transition
        // At 0°: cos(0) = 1, so factor = 0 (just carriage)
        // At 90°: cos(90°) = 0, so factor = 0.5 (half robot weight)
        // At 180°: cos(180°) = -1, so factor = 1 (full robot weight)
        double factor = (1 - Math.cos(Math.toRadians(currentPivotAngleDegrees))) / 2.0;
        
        return LIFT_CARRIAGE_MASS_KG + (ROBOT_MASS_KG * factor);
    }
    
    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        // Update pivot angle and check if we need to adjust lift mass
        double previousPivotAngle = currentPivotAngleDegrees;
        currentPivotAngleDegrees = getPivotAngleDegrees();
        
        // Recreate lift sim if pivot angle changed significantly (every 5 degrees)
        if (Math.abs(currentPivotAngleDegrees - previousPivotAngle) > 5.0) {
            double effectiveMass = calculateEffectiveLiftMass();
            recreateLiftSim(effectiveMass);
        }
        
        // Run position control if setpoint is active
        if (liftSetpointMeters != null) {
            double currentPosition = liftSim.getPositionMeters();
            double output = liftPID.calculate(currentPosition, liftSetpointMeters);
            liftAppliedVolts = MathUtil.clamp(output, -12.0, 12.0);
            liftSim.setInputVoltage(liftAppliedVolts);
        }
        
        if (pivotSetpointDegrees != null) {
            double currentAngle = getPivotAngleDegrees();
            double output = pivotPID.calculate(currentAngle, pivotSetpointDegrees);
            pivotAppliedVolts = MathUtil.clamp(output, -12.0, 12.0);
            pivotSim.setInputVoltage(pivotAppliedVolts);
        }
        
        // Update simulations
        liftSim.update(LOOP_PERIOD_SECS);
        pivotSim.update(LOOP_PERIOD_SECS);
        
        // Set lift inputs
        inputs.liftPositionMeters = liftSim.getPositionMeters();
        inputs.liftVelocityMetersPerSec = liftSim.getVelocityMetersPerSecond();
        inputs.liftAppliedVolts = liftAppliedVolts;
        inputs.liftCurrentAmps = liftSim.getCurrentDrawAmps();
        inputs.liftTempCelsius = 30.0 + (inputs.liftCurrentAmps * 0.5); // Simple temp model
        
        // Set pivot inputs
        inputs.pivotPositionDegrees = getPivotAngleDegrees();
        inputs.pivotVelocityDegreesPerSec = getPivotVelocityDegreesPerSec();
        inputs.pivotAppliedVolts = pivotAppliedVolts;
        inputs.pivotCurrentAmps = pivotSim.getCurrentDrawAmps();
        inputs.pivotTempCelsius = 30.0 + (inputs.pivotCurrentAmps * 0.5); // Simple temp model
        
        // Simulate limit switches
        inputs.liftBottomLimit = liftSim.getPositionMeters() <= ClimbConstants.kLiftMinHeight + 0.01;
        inputs.liftTopLimit = liftSim.getPositionMeters() >= ClimbConstants.kLiftMaxHeight - 0.01;
    }
    
    @Override
    public void setLiftPosition(double positionMeters) {
        liftSetpointMeters = MathUtil.clamp(
            positionMeters,
            ClimbConstants.kLiftMinHeight,
            ClimbConstants.kLiftMaxHeight
        );
    }
    
    @Override
    public void setPivotAngle(double angleDegrees) {
        pivotSetpointDegrees = MathUtil.clamp(
            angleDegrees,
            ClimbConstants.kPivotMinAngle,
            ClimbConstants.kPivotMaxAngle
        );
    }
    
    @Override
    public void setLiftVoltage(double volts) {
        liftSetpointMeters = null; // Disable position control
        liftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        liftSim.setInputVoltage(liftAppliedVolts);
    }
    
    @Override
    public void setPivotVoltage(double volts) {
        pivotSetpointDegrees = null; // Disable position control
        pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        pivotSim.setInputVoltage(pivotAppliedVolts);
    }
    
    @Override
    public void stop() {
        liftSetpointMeters = null;
        pivotSetpointDegrees = null;
        setLiftVoltage(0.0);
        setPivotVoltage(0.0);
    }
    
    @Override
    public void resetEncoders() {
        // Reset simulations to zero position
        liftSim.setState(0.0, 0.0);
        pivotSim.setState(0.0, 0.0);
    }
    
    @Override
    public void setBrakeMode(boolean enabled) {
        // Brake mode is simulated by stopping motors instantly when voltage is 0
        // This is handled automatically in the simulation
    }
    
    private double getPivotAngleDegrees() {
        // Convert radians to degrees
        return Math.toDegrees(pivotSim.getAngularPositionRad());
    }
    
    private double getPivotVelocityDegreesPerSec() {
        // Convert rad/s to deg/s
        return Math.toDegrees(pivotSim.getAngularVelocityRadPerSec());
    }
}