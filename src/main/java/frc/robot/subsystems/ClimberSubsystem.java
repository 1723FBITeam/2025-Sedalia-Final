package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor1 = new SparkMax(Constants.ClimberPorts.Motor1, MotorType.kBrushless);
    private final SparkMax climberMotor2 = new SparkMax(Constants.ClimberPorts.Motor2, MotorType.kBrushless);
    private final SparkMax clawMotor = new SparkMax(Constants.ClimberPorts.Claw, MotorType.kBrushless);

    private static final double MOTOR_POWER = 0.95;

    public ClimberSubsystem() {
    }

    public void climberDown() {
        climberMotor1.set(MOTOR_POWER);
        climberMotor2.set(-MOTOR_POWER);
    }

    public void climberUp() {
        climberMotor1.set(-MOTOR_POWER);
        climberMotor2.set(MOTOR_POWER);
    }

    public void clawClose() {
        clawMotor.set(.4);
    }

    public void clawOpen() {
        clawMotor.set(-.4);
    }

    public void stopMotor() {
        climberMotor1.set(0.0);
        climberMotor2.set(0.0);
    }

    public void stopClaw() {
        clawMotor.set(0.0);
    }

}