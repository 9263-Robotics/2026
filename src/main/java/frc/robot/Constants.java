// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleBinaryOperator;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
    public static final double DownPos = -16.1;
    public static final double MiddlePos = -5.5;
  }

  public static class ShooterLookupTables {
    public static InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();

    static {
      /* OLD HOOD DATA */
      // flywheelSpeedMap.put(0.0,-2500.0);
      // flywheelSpeedMap.put(0.946404,-2500.0);
      // flywheelSpeedMap.put(1.546404,-3500.0);
      // flywheelSpeedMap.put(2.146404,-3500.0);
      // flywheelSpeedMap.put(2.446404, -3500.0);
      // flywheelSpeedMap.put(3.296404,-3500.0);
      // flywheelSpeedMap.put(3.696404,-3700.0);
      // flywheelSpeedMap.put(4.696404,-4000.0);
      // flywheelSpeedMap.put(5.386404,-4250.0);
      // flywheelSpeedMap.put(6.896404,-5000.0);  
      
      // // This is made up:
      // flywheelSpeedMap.put(10.0,-5800.0);
      // flywheelSpeedMap.put(16.0, -6000.0);

      /* NEW HOOD DATA */
      // flywheelSpeedMap.put(0.0,-1900.0);
      // flywheelSpeedMap.put(1.046404,-1900.0);
      // flywheelSpeedMap.put(1.596404,-2000.0);
      // flywheelSpeedMap.put(2.216404,-2200.0);
      // flywheelSpeedMap.put(2.956404, -2450.0);
      // flywheelSpeedMap.put(3.426404,-2600.0);
      // flywheelSpeedMap.put(4.646404,-2850.0);
      // flywheelSpeedMap.put(5.506404,-3500.0);
      // flywheelSpeedMap.put(7.176404,-4300.0);
      // flywheelSpeedMap.put(8.146404, -5000.0);
      // flywheelSpeedMap.put(9.566404,-5250.0);
      // flywheelSpeedMap.put(11.096404, -5400.0);

      /* NEW NEW HOOD DATA */
      flywheelSpeedMap.put(0.0,-1900.0);
      flywheelSpeedMap.put(1.036404,-1900.0);
      flywheelSpeedMap.put(1.438404,-2000.0);
      flywheelSpeedMap.put(2.201404,-2300.0);
      flywheelSpeedMap.put(2.776404,-2600.0);
      flywheelSpeedMap.put(3.146404,-2700.0);
      flywheelSpeedMap.put(3.896404,-2700.0);
      flywheelSpeedMap.put(4.696404,-3100.0);
      flywheelSpeedMap.put(6.036404,-4000.0);
      flywheelSpeedMap.put(7.646404,-4700.0);
      flywheelSpeedMap.put(9.196404,-5000.0);


      // flywheelSpeedMap.put(0.0,-1900.0);
      // flywheelSpeedMap.put(1.036404,-1900.0);
      // flywheelSpeedMap.put(1.438404,-2000.0);
      // flywheelSpeedMap.put(2.201404,-2300.0);
      // flywheelSpeedMap.put(2.776404,-2600.0);
      // flywheelSpeedMap.put(3.146404,-2700.0);
      // flywheelSpeedMap.put(3.896404,-2700.0);
      // flywheelSpeedMap.put(4.696404,-3100.0);
      // flywheelSpeedMap.put(6.036404,-4000.0);
      // flywheelSpeedMap.put(7.646404,-4700.0);
      // flywheelSpeedMap.put(9.196404,-5000.0);

    }

    public static InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    static {
      /* OLD HOOD DATA */
      // hoodAngleMap.put(0.0,0.0);
      // hoodAngleMap.put(0.946404,0.0);
      // hoodAngleMap.put(1.546404,0.0);
      // hoodAngleMap.put(2.146404,0.0);
      // hoodAngleMap.put(2.446404, -1.0);
      // hoodAngleMap.put(3.296404,-4.0);
      // hoodAngleMap.put(3.696404,-6.0);
      // hoodAngleMap.put(4.696404,-8.0);
      // hoodAngleMap.put(5.386404,-12.0);
      // hoodAngleMap.put(6.896404,-17.0);

      // // this is made up:
      // hoodAngleMap.put(10.0,-23.0);
      // hoodAngleMap.put(16.0,-25.5);


      /* NEW HOOD DATA */
      // hoodAngleMap.put(0.0,0.0);
      // hoodAngleMap.put(1.046404,0.0);
      // hoodAngleMap.put(1.596404,-1.0);
      // hoodAngleMap.put(2.216404,-3.0);
      // hoodAngleMap.put(2.956404, -6.0);
      // hoodAngleMap.put(3.426404,-7.0);
      // hoodAngleMap.put(4.646404,-10.0);
      // hoodAngleMap.put(5.506404,-17.0);
      // hoodAngleMap.put(7.176404,-12.0);
      // hoodAngleMap.put(8.146404, -22.0);
      // hoodAngleMap.put(9.566404,-24.0);
      // hoodAngleMap.put(11.096404, -25.0);

      /* NEW NEW HOOD DATA */
      hoodAngleMap.put(0.0,0.0);
      hoodAngleMap.put(1.036404,0.0);
      hoodAngleMap.put(1.438404,0.0);
      hoodAngleMap.put(2.201404,0.0);
      hoodAngleMap.put(2.776404,-1.0);
      hoodAngleMap.put(3.146404,-2.0);
      hoodAngleMap.put(3.896404,-4.0);
      hoodAngleMap.put(4.696404,-7.0);
      hoodAngleMap.put(6.036404,-10.0);
      hoodAngleMap.put(7.646404,-15.0);
      hoodAngleMap.put(9.196404,-23.0);

      

    }

    public static InterpolatingDoubleTreeMap TOFMap = new InterpolatingDoubleTreeMap();
    static {
      TOFMap.put(0.0, 1.2);

      TOFMap.put(2.216404,1.25);
      TOFMap.put(2.956404, 1.51);
      TOFMap.put(3.426404,1.6);
      TOFMap.put(4.646404,1.76);
      TOFMap.put(5.506404,1.75);
      TOFMap.put(7.176404,1.47);

      TOFMap.put(10.0, 1.8);
    }
  }
}
