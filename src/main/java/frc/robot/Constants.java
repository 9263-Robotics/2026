// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.03;
  }

  public static class SwerveConstants {
    // max speed of the robot in m/s use Units.feetToMeters to use feet
    public static final double MAX_SPEED = Units.feetToMeters(16.5);

    // placeholders for turret offset
    public static final double TURRET_OFFSET_X = 0;
    public static final double TURRET_OFFSET_Y = 0;
  }


  public static class ScoringConstants {
    public static final double AIM_OFFSET_MULT = 1;

    public static final double ZeroSwitchPos = 0;
  }

  public static class CANIDs {

    // 0 --> RIO
    // 1-8 --> Swerve, from FL,FR,BL,BR, odd are spark maxs, even are krakenx60s, eg. FR is 3-SM, 4-Kx60
    // 20 --> Gyro
    
    public static final int PDH = 9;
    public static final int TurretMotor = 11;

    public static class TurretPID{
      public static final double k = 0.001;
      public static final double i = 0;
      public static final double d = 0;

      public static final double maxVel = 180;
      public static final double maxAccel = 540;
    }
  }
  public static class OuttakeConstants {
    public static final int MOTORID1 = 0; // placeholders
    public static final int MOTORID2 = 1;
    public static final double P = 0.075, I = 0.03, D = 0.005;
    public static final double ratio = 1.2;
    public static final double threshold = 2.0;
  }

  public static class Setpoints {
    public static class Hood { // placeholders
      public static final double STOWED = 0;
    }
    public static class IntakeArm { // placeholders
      public static final double STOWED = 90;
      public static final double ENGAGED = 0;
    }
  }
}
