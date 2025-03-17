package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HandSubsystem extends SubsystemBase {
    private final SparkFlex algaeMotor = new SparkFlex(Constants.ArmPorts.AlgaeMotor, MotorType.kBrushless);
    private final SparkFlex coralMoter = new SparkFlex(Constants.ArmPorts.CoralMotor, MotorType.kBrushless);

    private static final double MOTOR_POWER = .50; 

    public HandSubsystem() {
        
    }

    public void intakeAlgae() {
        algaeMotor.set(MOTOR_POWER);
        coralMoter.set(-MOTOR_POWER);
    }
    public void outputAlgaeProcessor() {
        algaeMotor.set(-MOTOR_POWER);
        coralMoter.set(MOTOR_POWER);
    }
    public void outputAlgaeBarge() {
        algaeMotor.set(-MOTOR_POWER*2);
        coralMoter.set(MOTOR_POWER*2);
    }
    public void stopMotor() {
        algaeMotor.set(0.08);
        coralMoter.set(-0.05);
    }

}