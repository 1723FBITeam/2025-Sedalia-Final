package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController; // Correct import for 2025+
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    private final SparkMax wristMotor = new SparkMax(Constants.ArmPorts.WristMotor, MotorType.kBrushed);
    private final RelativeEncoder wristEncoder = wristMotor.getEncoder();

    private static final double ENCODER_CPR = 2; // Encoder gives 2 ticks per 360° rotation
    private static final double DEGREES_PER_TICK = 360.0 / ENCODER_CPR; // Each tick = 180°
    private static final double TARGET_ANGLE_DEGREES = -90.0; // Target 90° rotation
    private static final double TARGET_TICKS = TARGET_ANGLE_DEGREES / DEGREES_PER_TICK; // Convert angle to ticks

    // PID Constants

    private static final double kP_BASE = 1; // Proportional gain
    private static final double kI = 0.0; // Integral gain
    private static final double kD = 0.0; // Derivative gain

    private PIDController pid;
    private double targetPosition;
    private boolean needToRotate = true;

    public WristSubsystem() {
        // Initialize PID controller with constants
        pid = new PIDController(kP_BASE, kI, kD);

        wristEncoder.setPosition(0);
    }

    public void toggleRotation() {
        needToRotate = !needToRotate;
        targetPosition = needToRotate ? TARGET_TICKS : 0; // Toggle between target position and zero
    }

    // TODO: verify these are good and not flipped
    public void wristHorizontal() {
        needToRotate = true;
        targetPosition = TARGET_TICKS; 
    }
    public void wristVertical() {
        needToRotate = false;
        targetPosition = 0; 
    }

    @Override
    public void periodic() {
        double currentPosition = wristEncoder.getPosition(); // Get current position from encoder
        double pidOutput = pid.calculate(currentPosition, targetPosition); // Calculate PID output

        // System.out.println("Encoder Position: " + currentPosition + " | Target: " + targetPosition + " | PID Output: "
        //         + pidOutput);

        wristMotor.set(pidOutput);

    }
}
