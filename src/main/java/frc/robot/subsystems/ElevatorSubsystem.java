package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorMotor1 = new TalonFX(Constants.ElevatorPorts.Motor1);
    private final TalonFX elevatorMotor2 = new TalonFX(Constants.ElevatorPorts.Motor2);



    public ElevatorSubsystem() {
        elevatorMotor1.setControl(new StaticBrake());
        elevatorMotor2.setControl(new StaticBrake());
    }

    public void elevatorUp(){
        
        elevatorMotor1.setControl(new DutyCycleOut(.2));
        elevatorMotor2.setControl(new DutyCycleOut(.2));
    }

    public void elevatorDown(){
        elevatorMotor1.setControl(new DutyCycleOut(-.05));
        elevatorMotor2.setControl(new DutyCycleOut(-.05));
    }

    public void stopMotor(){
        elevatorMotor1.setControl(new DutyCycleOut(.02));
        elevatorMotor2.setControl(new DutyCycleOut(.02));
    }
}