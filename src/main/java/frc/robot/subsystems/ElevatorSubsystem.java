package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorMotor1 = new TalonFX(Constants.ElevatorPorts.Motor1);
    private final TalonFX elevatorMotor2 = new TalonFX(Constants.ElevatorPorts.Motor2);

    private SlewRateLimiter rateLimiter = new SlewRateLimiter(3);
    // PID Constants
    private static final double kP_UP = 0.05; // Proportional gain
    private static final double kP_DOWN = 0.05; // Proportional gain
    private static final double kI = 0.0; // Integral gain
    private static final double kD = 0.0; // Derivative gain

    private PIDController pid;

    private boolean autoMode = false;
    private Map<Integer, Double> targetPositions = Map.of(
            0, 2.0,
            1, 10.0,
            2, 20.0
            );
    private int currentPositionKey = 0;

    public ElevatorSubsystem() {
        pid = new PIDController(kP_UP, kI, kD);
        elevatorMotor1.setControl(new StaticBrake());
        elevatorMotor2.setControl(new StaticBrake());
        elevatorMotor1.setPosition(0);
        elevatorMotor2.setPosition(0);
    }

    public void elevatorUpManual() {
        autoMode = false;
        elevatorMotor1.setControl(new DutyCycleOut(.3));
        elevatorMotor2.setControl(new DutyCycleOut(.3));
    }

    public void elevatorUpAuto() {
        autoMode = true;
        pid.setP(kP_UP);
        currentPositionKey = (currentPositionKey < 2) ? currentPositionKey + 1 : 2;
    }

    public void elevatorDownAuto() {
        autoMode = true;
        pid.setP(kP_DOWN);
        currentPositionKey = (currentPositionKey > 0) ? currentPositionKey - 1 : 0;

    }

    public void elevatorDownManual() {
        autoMode = false;
        elevatorMotor1.setControl(new DutyCycleOut(-.15));
        elevatorMotor2.setControl(new DutyCycleOut(-.15));
    }

    public void stopMotor() {
        elevatorMotor1.setControl(new DutyCycleOut(.03));
        elevatorMotor2.setControl(new DutyCycleOut(.03));
    }

    @Override
    public void periodic() {
        double currentPosition1 = elevatorMotor1.getPosition().getValueAsDouble();
        double currentPosition2 = elevatorMotor2.getPosition().getValueAsDouble(); // Get current position from encoder
        if (autoMode) {
            double pidOutput = pid.calculate(currentPosition1, targetPositions.get(currentPositionKey)); // Calculate PID output

            elevatorMotor1.set(rateLimiter.calculate(pidOutput));
            elevatorMotor2.set(rateLimiter.calculate(pidOutput));
        }

        System.out.println("Encoder Position1 : " + currentPosition1 + " | Target: " + targetPositions.get(currentPositionKey));
        System.out.println("Encoder Position2 : " + currentPosition2 + " | Target: " + targetPositions.get(currentPositionKey));
    }
}