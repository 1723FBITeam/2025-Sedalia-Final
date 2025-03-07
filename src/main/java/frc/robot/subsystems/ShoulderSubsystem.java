package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShoulderSubsystem extends SubsystemBase {
    private final SparkMax shoulderMotor = new SparkMax(Constants.ArmPorts.ShoulderMotor, MotorType.kBrushless);
    private static final double MOTOR_POWER = -.15;
    private boolean moving = false;

    public ShoulderSubsystem() {
        shoulderMotor.getEncoder().setPosition(0); 
    }

    public void shoulderUp() {
        shoulderMotor.set(MOTOR_POWER);
        moving = true;
    }

    public void shoulderDown() {
        shoulderMotor.set(-MOTOR_POWER);
        moving = true;
    }

    public void stopMotor() {
        moving = false;
    }

    @Override
    public void periodic() {
        
        if (!moving){
            if (shoulderMotor.getEncoder().getPosition() > 9.0) {
                shoulderMotor.set(MOTOR_POWER / 5);
            } else if (shoulderMotor.getEncoder().getPosition() > 2.0) {
                shoulderMotor.set(MOTOR_POWER / 6);
            } else {
                shoulderMotor.set(0);
            }
        }
        // System.out.println("shoulder: " + shoulderMotor.getEncoder().getPosition());
    }

}