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
    private static final double kP_UP = 0.05; // Proportional gain
    private static final double kP_DOWN = 0.05; // Proportional gain
    private static final double kI = 0.0; // Integral gain
    private static final double kD = 0.0; // Derivative gain

    private PIDController pid;

    private boolean autoMode = false;
    private Map<Integer, Double> targetPositions = Map.of(
            -1, -3.0,
            0, 0.0,
            1, 6.0);

    private int currentPositionKey = 0;

    public ShoulderSubsystem() {
        pid = new PIDController(kP_UP, kI, kD);
        shoulderMotor.getEncoder().setPosition(0);
    }

    public void shoulderDownAuto() {
        autoMode = true;
        pid.setP(kP_DOWN);
        currentPositionKey = (currentPositionKey < 1) ? currentPositionKey + 1 : 1;
    }

    public void shoulderUpAuto() {
        autoMode = true;
        pid.setP(kP_UP);
        currentPositionKey = (currentPositionKey > -1) ? currentPositionKey - 1 : -1;
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
        double currentPosition = shoulderMotor.getEncoder().getPosition();
        if (!moving && !autoMode) {
            if (shoulderMotor.getEncoder().getPosition() > 9.0) {
                shoulderMotor.set(MOTOR_POWER_HOLD);
            } else if (shoulderMotor.getEncoder().getPosition() > 2.0) {
                shoulderMotor.set(MOTOR_POWER_HOLD * .8);
            } else {
                shoulderMotor.set(0);
            }
        }
        if (autoMode) {
            double pidOutput = pid.calculate(currentPosition, targetPositions.get(currentPositionKey)); 
            shoulderMotor.set(rateLimiter.calculate(pidOutput));

        }

        SmartDashboard.putNumber("Shoulder ", shoulderMotor.getEncoder().getPosition());
        System.out.println("shoulder: " + shoulderMotor.getEncoder().getPosition());
    }

}