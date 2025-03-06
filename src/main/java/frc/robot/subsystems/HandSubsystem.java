package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HandSubsystem extends SubsystemBase {
    private final SparkFlex algaeMotor = new SparkFlex(Constants.ArmPorts.AlgaeMotor, MotorType.kBrushless);
    private final SparkFlex coralMoter = new SparkFlex(Constants.ArmPorts.CoralMotor, MotorType.kBrushless);

    private static final double MOTOR_POWER = .5; 

    public HandSubsystem() {

    }

    public void intakeAlgae() {
        algaeMotor.set(MOTOR_POWER);
        coralMoter.set(-MOTOR_POWER);
    }
    public void outputAlgae() {
        algaeMotor.set(-MOTOR_POWER);
        coralMoter.set(MOTOR_POWER);

    }
    public void intakeCoral() {
        coralMoter.set(MOTOR_POWER);
    }
    public void outputCoral() {
        coralMoter.set(-MOTOR_POWER);
    }

    public void stopMotor() {
        coralMoter.stopMotor();
        algaeMotor.stopMotor();
    }

}