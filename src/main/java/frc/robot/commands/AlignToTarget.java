package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command to align the robot to a Limelight target (rotation only).
 * Useful for aiming at a target without moving toward it.
 */
public class AlignToTarget extends Command {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    private final int priorityTagID;
    private final double maxAngularRate;
    
    private final double kP_aim = 0.008; // Proportional constant for rotation (reduced from 0.015)
    private final double maxRotationSpeed = 0.6; // Max rotation speed multiplier (60% of maxAngularRate)
    private final double aimTolerance = 2.0; // degrees
    
    private final SwerveRequest.RobotCentric driveRequest;
    
    /**
     * Creates a new AlignToTarget command.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param limelightName Name of the Limelight (e.g., "limelight")
     * @param priorityTagID AprilTag ID to prioritize (use -1 for any target)
     * @param maxAngularRate Maximum angular rate in radians per second
     */
    public AlignToTarget(CommandSwerveDrivetrain drivetrain, String limelightName, 
                        int priorityTagID, double maxAngularRate) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        this.priorityTagID = priorityTagID;
        this.maxAngularRate = maxAngularRate;
        this.driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        if (priorityTagID > 0) {
            LimelightHelpers.setPriorityTagID(limelightName, priorityTagID);
        }
    }
    
    @Override
    public void execute() {
        boolean tv = LimelightHelpers.getTV(limelightName);
        double tx = LimelightHelpers.getTX(limelightName);
        
        if (!tv) {
            drivetrain.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
            return;
        }
        
        // Calculate rotation rate (negative because tx is positive when target is right)
        double rotationRate = -tx * kP_aim * maxAngularRate;
        
        // Clamp rotation rate to max
        rotationRate = Math.max(-maxRotationSpeed * maxAngularRate, 
                               Math.min(maxRotationSpeed * maxAngularRate, rotationRate));
        
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(rotationRate));
    }
    
    @Override
    public boolean isFinished() {
        boolean tv = LimelightHelpers.getTV(limelightName);
        double tx = LimelightHelpers.getTX(limelightName);
        
        return tv && Math.abs(tx) < aimTolerance;
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }
}
