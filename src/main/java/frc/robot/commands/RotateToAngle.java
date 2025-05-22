package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotateToAngle {

    public static Command create(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive,
            double targetDegrees) {

        return new Command() {
            @Override
            public void initialize() {
            }

            @Override
            public void execute() {
                Rotation2d targetRotation = Rotation2d.fromDegrees(targetDegrees);
                System.out.println("ERGAERGAERGAERGAERGAERGAERG");
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(1.2));
            }

            @Override
            public boolean isFinished() {
                System.out.println("zxcvzxcvzxcvzxcvzxcvzxcvzxcv");
                double current = TunerConstants.pigeon2.getYaw().getValueAsDouble();
                double error = Math.abs(targetDegrees - current);
                return error < 3.0;
            }

            @Override
            public void end(boolean interrupted) {
                System.out.println("123123123123123123");
                drivetrain.setControl(
                        new SwerveRequest.FieldCentric()
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(0));
            }
        };
    }
}
