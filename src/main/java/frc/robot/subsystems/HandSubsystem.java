package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HandSubsystem extends SubsystemBase {
    private final SparkFlex algaeMotor = new SparkFlex(Constants.ArmPorts.AlgaeMotor, MotorType.kBrushless);
    private final SparkFlex coralMoter = new SparkFlex(Constants.ArmPorts.CoralMotor, MotorType.kBrushless);

    private static final double MOTOR_POWER_BARGE = 1.25;
    private static final double MOTOR_POWER_PROCESSOR = .35;
    private static final double MOTOR_POWER_INTAKE = .60; 

    public HandSubsystem() {
        
    }

    public void intakeAlgae() {
        algaeMotor.set(MOTOR_POWER_INTAKE);
        coralMoter.set(-MOTOR_POWER_INTAKE);
    }
    public void outputAlgaeProcessor() {
        algaeMotor.set(-MOTOR_POWER_PROCESSOR);
        coralMoter.set(MOTOR_POWER_PROCESSOR);
    }
    public void outputAlgaeBarge() {
        algaeMotor.set(-MOTOR_POWER_BARGE);
        coralMoter.set(MOTOR_POWER_BARGE);
    }
    public void stopMotor() {
        algaeMotor.set(0.08);
        coralMoter.set(-0.08);
    }

}