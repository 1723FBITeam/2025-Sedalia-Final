package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HandSubsystem extends SubsystemBase {
    private final SparkFlex algaeMotor = new SparkFlex(Constants.ArmPorts.AlgaeMotor, MotorType.kBrushless);
    private final SparkFlex coralMoter = new SparkFlex(Constants.ArmPorts.CoralMotor, MotorType.kBrushless);

    private static final double MOTOR_POWER = .6; 
    private static final double MOTOR_POWER_HOLD = .06; 

    public HandSubsystem() {

    }
    
    public void intakeCoral() {
        coralMoter.set(MOTOR_POWER);
    }
    public void outputCoral() {
        coralMoter.set(-MOTOR_POWER);
    }

    public void stopMotor() {
        coralMoter.set(MOTOR_POWER_HOLD);
        algaeMotor.set(MOTOR_POWER_HOLD);
    }

}