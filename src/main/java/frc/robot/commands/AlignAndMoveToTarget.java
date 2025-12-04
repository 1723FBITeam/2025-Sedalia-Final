package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command to align the robot to a Limelight target and drive toward it.
 * Uses proportional control for both rotation (alignment) and forward movement (ranging).
 */
public class AlignAndMoveToTarget extends Command {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    private final int priorityTagID;
    private final double targetTY; // Target vertical offset (when to stop)
    private final double maxSpeed;
    
    // PID constants - reduced for smoother movement
    private final double kP_aim = 0.008; // Proportional constant for rotation
    private final double kP_strafe = 0.005; // Proportional constant for strafing
    
    // Speed limits
    private final double minForwardSpeed = 0.15; // Minimum forward speed when close (15% of maxSpeed)
    private final double maxForwardSpeed = 0.6; // Maximum forward speed when far (60% of maxSpeed)
    private final double maxRotationSpeed = 0.4; // Max rotation speed multiplier (40% of maxAngularRate)
    
    // Tolerances
    private final double aimTolerance = 2.0; // degrees
    private final double targetAreaTolerance = 0.5; // ta units
    
    // Safety limits - stop if too close (either condition triggers stop)
    private final double tooCloseArea = 9.0; // Stop if area exceeds this (target too big - % of image)
    private final double tooCloseTY = 25.0; // Stop if ty exceeds this (target too high in frame)
    
    private final SwerveRequest.RobotCentric driveRequest;
    
    /**
     * Creates a new AlignAndMoveToTarget command.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param limelightName Name of the Limelight (e.g., "limelight")
     * @param priorityTagID AprilTag ID to prioritize (use -1 for any target)
     * @param targetArea Target area to stop at (typically 3-8 depending on distance)
     * @param maxSpeed Maximum speed in meters per second
     */
    public AlignAndMoveToTarget(CommandSwerveDrivetrain drivetrain, String limelightName, 
                                int priorityTagID, double targetArea, double maxSpeed) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        this.priorityTagID = priorityTagID;
        this.targetTY = targetArea; // Now using this as targetArea
        this.maxSpeed = maxSpeed;
        this.driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        // Set priority tag if specified
        if (priorityTagID > 0) {
            LimelightHelpers.setPriorityTagID(limelightName, priorityTagID);
        }
    }
    
    @Override
    public void execute() {
        // Get Limelight values
        boolean tv = LimelightHelpers.getTV(limelightName); // Valid target
        double tx = LimelightHelpers.getTX(limelightName); // Horizontal offset
        double ty = LimelightHelpers.getTY(limelightName); // Vertical offset
        double ta = LimelightHelpers.getTA(limelightName); // Target area (0-100%)
        
        // If no valid target or too close (EITHER too tall OR too big), stop
        if (!tv || ta > tooCloseArea || ty > tooCloseTY) {
            drivetrain.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
            return;
        }
        
        // Calculate rotation rate (aim at target)
        // Negative because tx is positive when target is to the right
        double rotationRate = -tx * kP_aim * maxSpeed;
        
        // Clamp rotation rate to max
        rotationRate = Math.max(-maxRotationSpeed * maxSpeed, 
                               Math.min(maxRotationSpeed * maxSpeed, rotationRate));
        
        // Calculate forward velocity with speed ramping based on distance
        double forwardVelocity = 0.0;
        if (ta < targetTY) { // targetTY is now targetArea
            // Calculate error (how far we are from target)
            double areaError = targetTY - ta;
            
            // Speed ramps based on distance: faster when far, slower when close
            // When area is small (far away), error is large -> faster
            // When area is large (close), error is small -> slower
            double speedScale = Math.min(1.0, areaError / targetTY);
            
            // Interpolate between min and max speed based on distance
            double targetSpeed = minForwardSpeed + (maxForwardSpeed - minForwardSpeed) * speedScale;
            
            // Apply speed with proportional control (NEGATIVE to move forward in robot-centric)
            forwardVelocity = -areaError * 0.15 * maxSpeed; // Base proportional control
            
            // Clamp to ramped speed limits (for negative values, max is closer to 0, min is more negative)
            forwardVelocity = Math.max(-targetSpeed * maxSpeed, 
                                      Math.min(-minForwardSpeed * maxSpeed, forwardVelocity));
        }
        
        // Optional: Calculate strafe velocity to center on target
        double strafeVelocity = -tx * kP_strafe * maxSpeed;
        
        // Apply the drive request
        drivetrain.setControl(driveRequest
            .withVelocityX(forwardVelocity)
            .withVelocityY(strafeVelocity)
            .withRotationalRate(rotationRate));
    }
    
    @Override
    public boolean isFinished() {
        boolean tv = LimelightHelpers.getTV(limelightName);
        double tx = LimelightHelpers.getTX(limelightName);
        double ty = LimelightHelpers.getTY(limelightName);
        double ta = LimelightHelpers.getTA(limelightName);
        
        // Finish when aligned and at target area, OR if too close (EITHER condition)
        boolean aligned = Math.abs(tx) < aimTolerance;
        boolean atTargetArea = Math.abs(ta - targetTY) < targetAreaTolerance;
        boolean tooClose = ta > tooCloseArea || ty > tooCloseTY;
        
        return (tv && aligned && atTargetArea) || tooClose;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }
}
