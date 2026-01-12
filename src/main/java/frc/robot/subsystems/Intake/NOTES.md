# Intake Subsystem

## Hardware
- **Pivot Motor**: SparkMax + NEO motor for pivoting over bumpers
  - Built-in NEO relative encoder for position feedback
  - Current limiting for stall protection
  
- **Roller Motor**: TBD for intaking game pieces (Write for sparks and talons)
  - Simple voltage control

## Software Architecture
Following AdvantageKit IO pattern:
- `IntakeIO` interface with AutoLog inputs
- `IntakeIOSparkMax` for real hardware (might be talon, victor, etc)
- `IntakeIOSim` for simulation
- `Intake` subsystem class

## Control Modes

### Pivot Control
- **Position Control**: Using SparkMax onboard PID
  - Stowed position: 0 degrees (starting/home position)
  - Deployed position: ~90-120 degrees (over bumpers)
  - Intermediate positions for different game pieces?
- **Safety Features**:
  - Soft limits to prevent over-rotation
  - Current limiting to detect jams
  - Manual override mode for recovery

### Roller Control  
- **Intake Mode**: Run at constant voltage/velocity (~8-12V)
- **Eject Mode**: Reverse voltage for stuck game pieces
- **Hold Mode**: Low power to maintain game piece grip
- **Auto-stop**: Current spike detection when game piece acquired

## Constants Required (all placeholders)
```java
// Add to Constants.java
public static class IntakeConstants {
    // Motor IDs
    public static final int kPivotMotorId = X;
    public static final int kRollerMotorId = X;
    
    // Positions (degrees)
    public static final double kStowedPosition = 0.0;
    public static final double kDeployedPosition = 90.0;
    
    // Speeds
    public static final double kIntakeVoltage = 8.0;
    public static final double kEjectVoltage = -4.0;
    
    // Current limits
    public static final int kPivotCurrentLimit = 30;
    public static final int kRollerCurrentLimit = 20;
    public static final double kRollerStallCurrent = 15.0;
    
    // PID values (tune these)
    public static final double kPivotP = 0.1;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.0;
}
```

## Commands Needed
- `DeployIntakeCommand` - Move to deployed position
- `StowIntakeCommand` - Return to stowed position  
- `IntakeGamePieceCommand` - Deploy + run rollers until current spike
- `EjectGamePieceCommand` - Run rollers in reverse
- `ManualIntakeControlCommand` - For testing/override