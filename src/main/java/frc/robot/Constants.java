// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class ControllerPorts {
    public static final int Driver = 0;
    public static final int Commander = 1;
  }
  
  public static class FieldConstants {
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.21;
    public static final double FIELD_CENTER_X = FIELD_LENGTH_METERS / 2.0;
    public static final double FIELD_CENTER_Y = FIELD_WIDTH_METERS / 2.0;
  }
  public static class ArmPorts {
    public static final int WristMotor = 16;
    public static final int ShoulderMotor = 17;
    public static final int AlgaeMotor = 19;
    public static final int CoralMotor = 18;
  }

  public static class ElevatorPorts {
    public static final int Motor1 = 11;
    public static final int Motor2 = 12;
  }

  public static class ClimberPorts {
    public static final int Motor1 = 14;
    public static final int Motor2 = 15;
    public static final int Claw = 20;
  }

  public static final int PowerDisHubPort = 13;



  public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

}
