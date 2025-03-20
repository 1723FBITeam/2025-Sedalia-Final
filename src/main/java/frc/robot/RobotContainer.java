// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.Constants.ControllerPorts;
// import frc.robot.commands.DropCoralCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);

        private final CommandXboxController m_driverController = new CommandXboxController(ControllerPorts.Driver);
        private final CommandXboxController m_commanderController = new CommandXboxController(
                        ControllerPorts.Commander);
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Controls");

        private final GenericEntry aButtonEntry = driverTab.add("A Button Pressed", false).getEntry();
        private final GenericEntry bButtonEntry = driverTab.add("B Button Pressed", false).getEntry();
        private final GenericEntry xButtonEntry = driverTab.add("X Button Pressed", false).getEntry();
        private final GenericEntry yButtonEntry = driverTab.add("Y Button Pressed", false).getEntry();

        private final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();
        private final HandSubsystem handSubsystem = new HandSubsystem();

        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

        private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

        /* Path follower */
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                NamedCommands.registerCommand("GrabAlgae", Commands.run(() -> {
                        handSubsystem.intakeAlgae();
                }, handSubsystem));
                NamedCommands.registerCommand("StopAlgae", Commands.run(() -> {
                        handSubsystem.stopMotor();
                }, handSubsystem));
                NamedCommands.registerCommand("OutputAlgae", Commands.run(() -> {
                        handSubsystem.outputAlgaeBarge();
                }, handSubsystem));
                NamedCommands.registerCommand("AlgaeDownTest", Commands.runOnce(() -> {
                        shoulderSubsystem.shoulderForward();
                        elevatorSubsystem.elevatorLevel1();
                }, elevatorSubsystem, shoulderSubsystem));
                NamedCommands.registerCommand("ClimberOut", Commands.runOnce(() -> {
                        climberSubsystem.climberDown();
                },  climberSubsystem));
                NamedCommands.registerCommand("ClimberStop", Commands.runOnce(() -> {
                        climberSubsystem.climberStop();
                },  climberSubsystem));
                new EventTrigger("AlgaeFinish").onTrue(Commands.runOnce(() -> {
                        elevatorSubsystem.elevatorBottom();
                        shoulderSubsystem.shoulderReset();
                }, elevatorSubsystem, shoulderSubsystem));
                new EventTrigger("setLevel1Algae").onTrue(Commands.runOnce(() -> {
                        elevatorSubsystem.elevatorLevel1();
                        shoulderSubsystem.shoulderForward();
                }, elevatorSubsystem, shoulderSubsystem));
                new EventTrigger("setLevel2Algae").onTrue(Commands.runOnce(() -> {
                        elevatorSubsystem.elevatorLevel2();
                        shoulderSubsystem.shoulderForward();
                }, elevatorSubsystem, shoulderSubsystem));
                new EventTrigger("BargeScore").onTrue(Commands.runOnce(() -> {
                        elevatorSubsystem.elevatorTop();
                        shoulderSubsystem.shoulderBackward();
                }, elevatorSubsystem, shoulderSubsystem ));





                autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("Auto Mode", autoChooser);

                configureBindings();
                startButtonUpdater();
                // setupCamera();
                setLimelight();
        }

        private void setLimelight() {
                new Thread(() -> {
                        while (true) {
                                // Continuously update Shuffleboard button states
                                // Get the Limelight NetworkTable
                                NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
                                NetworkTableEntry tx = table.getEntry("tx");
                                NetworkTableEntry ty = table.getEntry("ty");
                                NetworkTableEntry ta = table.getEntry("ta");
                                NetworkTableEntry tid = table.getEntry("tid");

                                // Read values from Limelight
                                double x = tx.getDouble(0.0); // Horizontal offset from crosshair to target
                                double y = ty.getDouble(0.0); // Vertical offset from crosshair to target
                                double area = ta.getDouble(0.0); // Target area (0 to 100%)
                                double id = tid.getDouble(0.0);
                                // Post values to Shuffleboard
                                SmartDashboard.putNumber("Limelight Tag ID", id);
                                SmartDashboard.putNumber("LimelightX", x);
                                SmartDashboard.putNumber("LimelightY", y);
                                SmartDashboard.putNumber("LimelightArea", area);
                                // Sleep for a short duration to prevent overloading CPU
                                Timer.delay(0.05); // 50ms delay
                        }
                }).start();
        }

        private void startButtonUpdater() {
                new Thread(() -> {
                        while (true) {
                                // Continuously update Shuffleboard button states
                                aButtonEntry.setBoolean(m_driverController.a().getAsBoolean());
                                bButtonEntry.setBoolean(m_driverController.b().getAsBoolean());
                                xButtonEntry.setBoolean(m_driverController.x().getAsBoolean());
                                yButtonEntry.setBoolean(m_driverController.y().getAsBoolean());

                                // Sleep for a short duration to prevent overloading CPU
                                Timer.delay(0.05); // 50ms delay
                        }
                }).start();
        }

        private void setupCamera() {
                UsbCamera camera = CameraServer.startAutomaticCapture();
                camera.setResolution(500, 300);
                camera.setFPS(15);
        }

        // THIS IS WHERE YOU CHANGE THE SPEED!!!
        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(xLimiter
                                                                .calculate(-joystick.getLeftY() * 0.7 * MaxSpeed))
                                                .withVelocityY(yLimiter
                                                                .calculate(-joystick.getLeftX() * 0.7 * MaxSpeed))
                                                .withRotationalRate(rotationLimiter.calculate(
                                                                -joystick.getRightX() * 0.85 * MaxAngularRate))));

                // m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // joystick.b().whileTrue(drivetrain.applyRequest(() ->
                // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
                // -joystick.getLeftX()))
                // ));

                m_driverController.pov(0).whileTrue(drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(0.5).withVelocityY(0))
                );
                m_driverController.pov(180).whileTrue(drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(-0.5).withVelocityY(0))
                );
                m_driverController.pov(90).whileTrue(drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(0).withVelocityY(-0.5))
                );
                m_driverController.pov(270).whileTrue(drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(0).withVelocityY(0.5))
                );

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on left bumper press
                m_driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                drivetrain.registerTelemetry(logger::telemeterize);

                // m_driverController.a().whileTrue(new StartEndCommand(
                // wristSubsystem::toggleRotation,
                // () -> {
                // },
                // wristSubsystem));

                m_driverController.x().whileTrue(new StartEndCommand(
                                handSubsystem::intakeAlgae,
                                handSubsystem::stopMotor,
                                handSubsystem));

                m_driverController.b().whileTrue(new StartEndCommand(
                                handSubsystem::outputAlgaeProcessor,
                                handSubsystem::stopMotor,
                                handSubsystem));

                m_driverController.y().whileTrue(new StartEndCommand(
                                handSubsystem::outputAlgaeBarge,
                                handSubsystem::stopMotor,
                                handSubsystem));

                m_driverController.leftBumper().whileTrue(new StartEndCommand(
                                shoulderSubsystem::shoulderUp,
                                shoulderSubsystem::stopMotor,
                                shoulderSubsystem));

                m_driverController.leftTrigger().whileTrue(new StartEndCommand(
                                shoulderSubsystem::shoulderDown,
                                shoulderSubsystem::stopMotor,
                                shoulderSubsystem));
                m_driverController.rightBumper().whileTrue(new StartEndCommand(
                                elevatorSubsystem::elevatorUpManual,
                                elevatorSubsystem::stopMotor,
                                elevatorSubsystem));

                m_driverController.rightTrigger().whileTrue(new StartEndCommand(
                                elevatorSubsystem::elevatorDownManual,
                                elevatorSubsystem::stopMotor,
                                elevatorSubsystem));

                m_commanderController.start().whileTrue(new StartEndCommand(
                                climberSubsystem::climberUp,
                                climberSubsystem::climberStop,
                                climberSubsystem));

                m_commanderController.back().whileTrue(new StartEndCommand(
                                climberSubsystem::climberDown,
                                climberSubsystem::climberStop,
                                climberSubsystem));

                m_commanderController.y().whileTrue(new StartEndCommand(
                                () -> {
                                        elevatorSubsystem.elevatorTop();
                                        shoulderSubsystem.shoulderBackward();
                                },
                                handSubsystem::stopMotor,
                                handSubsystem, shoulderSubsystem, elevatorSubsystem));

                m_commanderController.a().whileTrue(new StartEndCommand(
                                () -> {
                                        elevatorSubsystem.elevatorBottom();
                                        shoulderSubsystem.shoulderForward();
                                },
                                handSubsystem::stopMotor,
                                handSubsystem, shoulderSubsystem, elevatorSubsystem));

                m_commanderController.b().whileTrue(new StartEndCommand(
                                () -> {
                                        elevatorSubsystem.elevatorLevel2();
                                        shoulderSubsystem.shoulderForward();
                                },
                                handSubsystem::stopMotor,
                                handSubsystem, shoulderSubsystem, elevatorSubsystem));

                m_commanderController.x().whileTrue(new StartEndCommand(
                                () -> {
                                        elevatorSubsystem.elevatorLevel1();
                                        shoulderSubsystem.shoulderForward();
                                },
                                handSubsystem::stopMotor,
                                handSubsystem, shoulderSubsystem, elevatorSubsystem));

                m_commanderController.leftBumper().whileTrue(new StartEndCommand(
                                shoulderSubsystem::shoulderUpAuto,
                                () -> {
                                },
                                shoulderSubsystem));

                m_commanderController.leftTrigger().whileTrue(new StartEndCommand(
                                shoulderSubsystem::shoulderDownAuto,
                                () -> {
                                },
                                shoulderSubsystem));
                m_commanderController.rightBumper().whileTrue(new StartEndCommand(
                                elevatorSubsystem::elevatorUpAuto,
                                () -> {
                                },
                                elevatorSubsystem));

                m_commanderController.rightTrigger().whileTrue(new StartEndCommand(
                                elevatorSubsystem::elevatorDownAuto,
                                () -> {
                                },
                                elevatorSubsystem));
        }

        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return autoChooser.getSelected();
        }
}
