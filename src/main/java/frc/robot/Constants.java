// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static class DriverController {
        public static final int PORT = 0;
    }

    public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }


  public static final double loopPeriodSecs = 0.02;

  public static class FieldPoses {
      public static final Translation2d blueHub = new Translation2d(4.629, 4.014);
      public static final Translation2d redHub = new Translation2d(11.943, 4.014);
  }

  public static class Shooter {
      public static final InterpolatingDoubleTreeMap simHoodAngleInterpolationMap = new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap simFlywheelVelocityInterpolationMap = new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap hoodAngleInterpolationMap = new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap flywheelVelocityInterpolationMap = new InterpolatingDoubleTreeMap();
      public static double amperageThreshold = 55;

      static {
        simHoodAngleInterpolationMap.put(1.635, Units.degreesToRadians(78));
        simHoodAngleInterpolationMap.put(4.5, Units.degreesToRadians(72));

        simFlywheelVelocityInterpolationMap.put(1.635, 38.0);
        simFlywheelVelocityInterpolationMap.put(2.0, 43.0);
        simFlywheelVelocityInterpolationMap.put(3.0, 48.0);
        simFlywheelVelocityInterpolationMap.put(4.5, 50.0);
      }

      // meters
      public static final double flywheelDiameter = 0.1016;

      public static class Turret {
          public static final double simP = 0.5;
          public static final double simI = 0;
          public static final double simD = 0.05;

          public static final double leftLimit = Math.PI;
          public static final double rightLimit = -Math.PI;
      }
      public static class Hood {
          public static final double simP = 0.5;
          public static final double simI = 0;
          public static final double simD = 0.05;
      }
      public static class Flywheel{
          public static final double simP = 10.5;
          public static final double simI = 0;
          public static final double simD = 0.0;
          public static final double simS = 8;
      }
      public static class Feeder{
          public static final double simP = 6;
          public static final double simI = 0;
          public static final double simD = 0.0;
          public static final double simS = 0.0;
      }
  }
}
