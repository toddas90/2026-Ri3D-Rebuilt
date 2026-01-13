package frc.robot.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    
    // Simulated motors
    private final DCMotorSim armMotorSim;
    private final DCMotorSim rollerMotorSim;

    // PID controller for arm position
    private final PIDController armPID;

    // Control mode tracking
    private Double armSetpointDegrees = null;

    //applied voltages
    private double armAppliedVolts = 0.0;
    private double rollerAppliedVolts = 0.0;

    // Simulation constants
    private static final double MOI = 0.01; // Moment of inertia for arm


    public IntakeIOSim() {
        armMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getBag(1), 
                MOI, 
                IntakeConstants.kArmGearRatio
            ),
            DCMotor.getBag(1)
        );

        rollerMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getVex775Pro(1), MOI, IntakeConstants.kRollerGearRatio),
            DCMotor.getCIM(1)
        );

        armPID = new PIDController(
            IntakeConstants.kArmP,
            IntakeConstants.kArmI, 
            IntakeConstants.kArmD
        ); // Example PID values
        armPID.setTolerance(IntakeConstants.kArmPositionTolerance); // 2 degree tolerance
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        // Update arm control
        if (armSetpointDegrees != null) {
            double armOutput = armPID.calculate(inputs.armPositionDegrees, armSetpointDegrees);
            armAppliedVolts = Math.max(-12.0, Math.min(12.0, armOutput));
        } else {
            armAppliedVolts = 0.0;
        }

        // Update simulations
        armMotorSim.setInputVoltage(armAppliedVolts);
        armMotorSim.update(0.02); // 20ms update

        rollerMotorSim.setInputVoltage(rollerAppliedVolts);
        rollerMotorSim.update(0.02); // 20ms update

        // Set arm inputs
        inputs.armPositionDegrees = armMotorSim.getAngularPositionRotations() * 360.0; // Convert to degrees
        inputs.armAppliedCurrentAmps = armMotorSim.getCurrentDrawAmps();
        inputs.armAppliedVolts = armAppliedVolts;
        inputs.armSetPointDegrees = (armSetpointDegrees != null) ? armSetpointDegrees : inputs.armPositionDegrees;

        // Set roller  inputs
        inputs.rollerAppliedCurrentAmps = rollerMotorSim.getCurrentDrawAmps();
        inputs.rollerAppliedVolts = rollerAppliedVolts;
    }

    @Override
    public void setArmPosition(double angleDegrees) {
        armSetpointDegrees = MathUtil.clamp(
            angleDegrees,
            IntakeConstants.kInPosition,
            IntakeConstants.kOutPosition
        );
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        rollerMotorSim.setInputVoltage(voltage);
    }

    @Override
    public void stop() {
        armSetpointDegrees = null;
        armAppliedVolts = 0.0;
        setRollerVoltage(0);
        armMotorSim.setInputVoltage(armAppliedVolts);
    }

    @Override
    public void resetEncoders() {
        armMotorSim.setState(0.0, 0.0);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        // Brake mode is simulated by stopping motors instantly when voltage is 0
        // This is handled automatically in the simulation
    }

}
