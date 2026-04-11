// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleBinaryOperator;

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
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.03;
  }

  public static class SwerveConstants {
    // max speed of the robot in m/s use Units.feetToMeters to use feet
    public static final double MAX_SPEED = Units.feetToMeters(20
    );
  }

  public static class TurretConstants {
    // 230 deg of rotation
    public static final double minSoftStop = -111;
    public static final double maxSoftStop = 119;

    public static final double AbsMinSoftStop = -116;
    public static final double AbsMaxSoftStop = 124;

  }
  public static class IntakeConstants {
    public static final double UpPos = 0;
    public static final double DownPos = -14.5;
    public static final double MiddlePos = -5.5;
  }
}
