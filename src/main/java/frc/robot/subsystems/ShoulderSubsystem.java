package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShoulderSubsystem extends SubsystemBase {
    private final SparkMax shoulderMotor = new SparkMax(Constants.ArmPorts.ShoulderMotor, MotorType.kBrushless);
    private static final double MOTOR_POWER = -.5;
    private static final double MOTOR_POWER_HOLD = -.04;
    private boolean moving = false;

    private SlewRateLimiter rateLimiter = new SlewRateLimiter(3);

    // PID Constants
    private static final double kP_UP = 0.05;
    private static final double kP_DOWN = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private PIDController pid;
    private boolean autoMode = false;

    private Map<Integer, Double> targetPositions = Map.of(
            -1, 15.0,
            0, 0.0,
            1, -7.8);

    private int currentPositionKey = 0;

    public ShoulderSubsystem() {
        pid = new PIDController(kP_UP, kI, kD);
        shoulderMotor.getEncoder().setPosition(0);
    }

    public void shoulderUpAuto() {
        autoMode = true;
        pid.setP(kP_UP);
        currentPositionKey = targetPositions.keySet().stream()
                .filter(k -> k > currentPositionKey)
                .min(Integer::compareTo)
                .orElse(currentPositionKey);
    }

    public void shoulderDownAuto() {
        autoMode = true;
        pid.setP(kP_DOWN);
        currentPositionKey = targetPositions.keySet().stream()
                .filter(k -> k < currentPositionKey)
                .max(Integer::compareTo)
                .orElse(currentPositionKey);
    }

    public void shoulderBackward() {
        autoMode = true;
        pid.setP(kP_UP);
        currentPositionKey = targetPositions.keySet().stream()
                .max(Integer::compareTo)
                .orElse(currentPositionKey);
    }

    public void shoulderForward() {
        autoMode = true;
        pid.setP(kP_DOWN);
        currentPositionKey = targetPositions.keySet().stream()
                .min(Integer::compareTo)
                .orElse(currentPositionKey);
    }

    public void shoulderUp() {
        autoMode = false;
        shoulderMotor.set(MOTOR_POWER);
        moving = true;
    }

    public void shoulderDown() {
        autoMode = false;
        shoulderMotor.set(-MOTOR_POWER);
        moving = true;
    }

    public void stopMotor() {
        moving = false;
    }

    @Override
    public void periodic() {
        double currentPosition = shoulderMotor.getEncoder().getPosition();

        if (!moving && !autoMode) {
            if (currentPosition > 10.0) {
                shoulderMotor.set(MOTOR_POWER_HOLD);
            } else if (currentPosition > 3.0) {
                shoulderMotor.set(MOTOR_POWER_HOLD * .8);
            } else {
                shoulderMotor.set(0);
            }
        }

        if (autoMode) {
            double pidOutput = pid.calculate(currentPosition, targetPositions.get(currentPositionKey));
            shoulderMotor.set(rateLimiter.calculate(pidOutput));
        }

        SmartDashboard.putNumber("Shoulder ", currentPosition);
        // System.out.println("Shoulder: " + currentPosition + " | Target: " + targetPositions.get(currentPositionKey));
    }
}