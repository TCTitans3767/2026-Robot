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

  public static final int ServoHubCANID = 1;

  public static final double loopPeriodSecs = 0.02;

  public static class FieldPoses {
      public static final Translation2d blueHub = new Translation2d(4.629, 4.014);
      public static final Translation2d redHub = new Translation2d(11.943, 4.014);

      public static final Translation2d blueLeftPassTarget = new Translation2d(1.35, 6.75);
      public static final Translation2d blueRightPassTarget = new Translation2d(1.35, 1.75);

      public static final Translation2d redLeftPassTarget = new Translation2d(15.1, 1.75);
      public static final Translation2d redRightPassTarget = new Translation2d(15.1, 6.75);
  }

  public static class Shooter {
      public static final InterpolatingDoubleTreeMap simHoodAngleInterpolationMap = new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap simFlywheelVelocityInterpolationMap = new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap hoodAngleInterpolationMap = new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap flywheelVelocityInterpolationMap = new InterpolatingDoubleTreeMap();
      public static double amperageThreshold = 55;
      public static double signalUpdateFrequency = 50;
      public static double turretLeadCorrectionConstant = 0.05;

      static {
        simHoodAngleInterpolationMap.put(1.635, Units.degreesToRadians(78));
        simHoodAngleInterpolationMap.put(4.5, Units.degreesToRadians(72));

        simFlywheelVelocityInterpolationMap.put(1.635, 38.0);
        simFlywheelVelocityInterpolationMap.put(2.0, 42.0);
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
          public static double currentLimit = 60;

          public static double motionMagicCruise = 100;
          public static double motionMagicAccel = 50;

          public static final double compP = 0;
          public static final double compI = 0;
          public static final double compD = 0;
          public static final double compS = 0;

          public static final double gearRatio = 18.75;

          // radians per second
          public static final double velocityLimit = 5;

          public static final double GEAR_0_TOOTH_COUNT = 110;
          public static final double GEAR_1_TOOTH_COUNT = 25;
          public static final double GEAR_2_TOOTH_COUNT = 24.0;
          public static final double SLOPE = (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
                  / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);
      }
      public static class Hood {
          public static final double simP = 0.5;
          public static final double simI = 0;
          public static final double simD = 0.05;

          public static final int minimumPulseWidth = 1000;
          public static final int maximumPulseWidth = 2000;
      }
      public static class Flywheel{
          public static final double simP = 10.5;
          public static final double simI = 0;
          public static final double simD = 0.0;
          public static final double simS = 8;

          public static double motionMagicCruise = 100;
          public static double motionMagicAccel = 50;

          public static final double compP = 0;
          public static final double compI = 0;
          public static final double compD = 0;
          public static final double compS = 0;

          public static final double gearRatio = 1;
          public static double currentLimit = 60;
      }
      public static class Feeder{
          public static final double simP = 6;
          public static final double simI = 0;
          public static final double simD = 0.0;
          public static final double simS = 0.0;

          public static double motionMagicCruise = 100;
          public static double motionMagicAccel = 50;

          public static final double compP = 0;
          public static final double compI = 0;
          public static final double compD = 0;
          public static final double compS = 0;

          public static final double gearRatio = 1;
          public static double currentLimit = 60;
      }
  }

  public static class Indexer {
      // Sim
      public static final double simP = 0.0;
      public static final double simI = 0.0;
      public static final double simD = 0.0;
      public static final double simS = 0.0;

      // Competition bot
      public static final double compP = 0.0;
      public static final double compI = 0.0;
      public static final double compD = 0.0;
      public static final double compS = 0.0;

      public static final int indexerCanID = 67;

      public static final double rollerCurrentLimit = 60;
      public static final double rollerConversionFactor = 1;

      public static final double rollerMaxVelocity = 10;
      public static final double rollerMaxAcceleration = 10;

      public static final double statusUpdateFrequency = 50;


  }
  public static class Intake{
      public static final int rollerMotorCANID = 0;
      public static final double RollerCurrentLimit = 0.0;
      public static final double RollerGearRatio = 0.0;
      public static final double RollerMotionMagicCruise = 0.0;
      public static final double RollerMotionMagicAccel = 0.0;
      public static final double RollerkP = 0.0;
      public static final double RollerkI = 0.0;
      public static final double RollerkD = 0.0;
      public static final double RollerkS = 0.0;
      public static final double RollerkG = 0.0;

      public static final int pivotMotorCANID = 0;
      public static final double PivotCurrentLimit = 0.0;
      public static final double PivotGearRatio = 0.0;
      public static final double PivotMotionMagicCruise = 0.0;
      public static final double PivotMotionMagicAccel = 0.0;
      public static final double PivotkP = 0.0;
      public static final double PivotkI = 0.0;
      public static final double PivotkD = 0.0;
      public static final double PivotkS = 0.0;
      public static final double PivotkG = 0.0;

      public static final double FrequencyUpdateRate = 0.0;
    }

}
