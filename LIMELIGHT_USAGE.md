# Limelight Alignment Commands - Usage Guide

## Commands Created

### 1. AlignAndMoveToTarget
Aligns the robot to a target AND moves toward it until reaching a specified distance.

**Features:**
- Rotates to face the target (using tx)
- Drives forward toward the target (using ty)
- Optional strafing to center on target
- Stops when aligned and at target distance

**Usage in RobotContainer:**
```java
m_driverController.a().whileTrue(
    new AlignAndMoveToTarget(drivetrain, "limelight", 12, 19.0, MaxSpeed)
);
```

**Parameters:**
- `drivetrain`: Your CommandSwerveDrivetrain
- `"limelight"`: Name of your Limelight
- `12`: AprilTag ID to target (use -1 for any target)
- `19.0`: Target ty value (vertical offset when you want to stop)
- `MaxSpeed`: Maximum speed in m/s

### 2. AlignToTarget
Only aligns (rotates) to face the target without moving forward.

**Features:**
- Rotates to face the target
- Does NOT move forward
- Useful for aiming before shooting

**Usage in RobotContainer:**
```java
m_driverController.b().whileTrue(
    new AlignToTarget(drivetrain, "limelight", 14, MaxAngularRate)
);
```

## Current Button Mappings

- **A Button**: Align and move to AprilTag ID 12, stop at ty=19.0
- **B Button**: Align to AprilTag ID 14 (rotation only)

## Tuning Guide

### If the robot oscillates (shakes back and forth):
1. **Reduce kP_aim** in the command files (currently 0.015)
   - Try 0.01 or 0.008
2. **Reduce kP_range** (currently 0.1)
   - Try 0.08 or 0.05

### If the robot is too slow to respond:
1. **Increase kP_aim** (currently 0.015)
   - Try 0.02 or 0.025
2. **Increase kP_range** (currently 0.1)
   - Try 0.12 or 0.15

### If the robot stops too far/close:
1. **Adjust targetTY** parameter when creating the command
   - Higher ty = closer to target
   - Lower ty = farther from target
   - Typical range: 15-25

### Tolerance Adjustments:
In the command files, you can adjust:
- `aimTolerance`: How accurate the alignment needs to be (default 2.0 degrees)
- `rangeTolerance`: How close to target distance (default 1.0 ty units)

## Testing Tips

1. **Start with alignment only** (B button) to tune rotation
2. **Then test full approach** (A button) to tune forward movement
3. **Monitor SmartDashboard** - Your existing code already publishes:
   - LimelightX (tx)
   - LimelightY (ty)
   - LimelightArea (ta)
   - Limelight Tag ID

4. **Check for valid targets**:
   - Make sure your Limelight can see AprilTags
   - Verify the tag IDs match what you're targeting
   - Check pipeline settings in Limelight web interface

## Advanced: Using Different Targets

To target different AprilTags for different actions:

```java
// Align to tag 1
m_driverController.leftBumper().whileTrue(
    new AlignAndMoveToTarget(drivetrain, "limelight", 1, 20.0, MaxSpeed)
);

// Align to tag 5
m_driverController.rightBumper().whileTrue(
    new AlignAndMoveToTarget(drivetrain, "limelight", 5, 18.0, MaxSpeed)
);

// Align to ANY visible target (use -1)
m_driverController.start().whileTrue(
    new AlignToTarget(drivetrain, "limelight", -1, MaxAngularRate)
);
```

## Troubleshooting

### Robot doesn't move:
- Check if Limelight has a valid target (tv should be 1)
- Verify Limelight NetworkTables connection
- Check that the correct pipeline is active

### Robot moves in wrong direction:
- The commands use RobotCentric control
- X is forward/backward
- Y is left/right (strafe)
- If directions are wrong, adjust the signs in the command

### Robot spins in circles:
- Reduce kP_aim significantly
- Check that tx values are reasonable (-30 to +30 degrees)

## Your Existing Code

Your old proportional control methods are still in RobotContainer:
- `limelight_aim_proportional()`
- `limelight_range_proportional()`

These are now replaced by the new commands, but you can keep them as reference or for manual control.
