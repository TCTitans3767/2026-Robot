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

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.ArrayList;
import java.util.List;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double shiftOffset = 1.0;
  public static CANBus SuperstructureCANBus = new CANBus("rio");

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

  public static final int ServoHubCANID = 45;

  public static final double loopPeriodSecs = 0.02;

  public static class FieldPoses {
      public static final Translation2d blueHub = new Translation2d(4.629, 4.014);
      public static final Translation2d redHub = new Translation2d(11.943, 4.014);

      public static final Translation2d blueLeftPassTarget = new Translation2d(1.35, 6.75);
      public static final Translation2d blueRightPassTarget = new Translation2d(1.35, 1.75);

      public static final Translation2d redLeftPassTarget = new Translation2d(15.1, 1.75);
      public static final Translation2d redRightPassTarget = new Translation2d(15.1, 6.75);

      public static final List<List<Translation2d>> noShootZones = new ArrayList<>();

      static {
          // Blue Left Trench
          noShootZones.add(List.of(new Translation2d(3.425, 8.180), new Translation2d(5.65, 6.7)));

          // Blue Right Trench
          noShootZones.add(List.of(new Translation2d(3.425, 1.36), new Translation2d(5.65, 0)));

          // Red Right Trench
          noShootZones.add(List.of(new Translation2d(10.7, 8.18), new Translation2d(13.125, 6.7)));

          // Red Left Trench
          noShootZones.add(List.of(new Translation2d(10.7, 1.36), new Translation2d(13.125, 0)));
      }
  }

  public static class Limelights {
      public static final int minMegaTagOneEstimations = 30;
      public static final double maxAmbiguity = 0.35;
  }

  public static class Shooter {
      public static final InterpolatingDoubleTreeMap simHoodAngleInterpolationMap = new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap simFlywheelVelocityInterpolationMap = new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap hoodAngleInterpolationMap = new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap flywheelVelocityInterpolationMap = new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap turretSpringFeedforwardInterpolationMap = new InterpolatingDoubleTreeMap();
      public static double amperageThreshold = 55;
      public static double signalUpdateFrequency = 20;
      public static double turretLeadCorrectionConstant = 0.05;

      static {
        simHoodAngleInterpolationMap.put(1.635, Units.degreesToRadians(78));
        simHoodAngleInterpolationMap.put(4.5, Units.degreesToRadians(72));

        simFlywheelVelocityInterpolationMap.put(1.635, 38.0);
        simFlywheelVelocityInterpolationMap.put(2.0, 42.0);
        simFlywheelVelocityInterpolationMap.put(3.0, 48.0);
        simFlywheelVelocityInterpolationMap.put(4.5, 50.0);

        turretSpringFeedforwardInterpolationMap.put(0.0, 3.0);
      }

      // meters
      public static final double flywheelDiameter = 0.1016;

      public static class Turret {
          public static final double simP = 0.5;
          public static final double simI = 0;
          public static final double simD = 0.05;

          public static final double leftLimit = 0;
          public static final double rightLimit = 0.96575;
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

          public static final int leftTurretMotorCANID = 43;
          public static final int rightTurretMotorCANID = 42;
      }
      public static class Hood {
          public static final double simP = 0.5;
          public static final double simI = 0;
          public static final double simD = 0.05;

          public static final int minimumPulseWidth = 1000;
          public static final int maximumPulseWidth = 2000;

          public static final int leftHoodServoHubPort = 2;
          public static final int rightHoodServoHubPort = 0;
      }
      public static class Flywheel{
          public static final double simP = 10.5;
          public static final double simI = 0;
          public static final double simD = 0.0;
          public static final double simS = 8;

          public static double motionMagicCruise = 400;
          public static double motionMagicAccel = 600;

          public static final double compP = 4.5;
          public static final double compI = 0;
          public static final double compD = 0;
          public static final double compS = 6;
          public static final double compV = 0.1;

          public static final double gearRatio = 0.75;
          public static double currentLimit = 60;

          public static final int leftFlywheelMotorCANID = 19;
          public static final int rightFlywheelMotorCANID = 18;
      }
      public static class Feeder{
          public static final double simP = 6;
          public static final double simI = 0;
          public static final double simD = 0.0;
          public static final double simS = 0.0;

          public static double motionMagicCruise = 100;
          public static double motionMagicAccel = 50;

          public static final double compP = 14;
          public static final double compI = 0;
          public static final double compD = 0;
          public static final double compS = 15;

          public static final double gearRatio = 1.67;
          public static double currentLimit = 60;

          public static final int leftFeederMotorCANID = 20;
          public static final int rightFeederMotorCANID = 44;
      }
  }

  public static class Indexer {
      // Sim
      public static final double simP = 0.0;
      public static final double simI = 0.0;
      public static final double simD = 0.0;
      public static final double simS = 0.0;

      // Competition bot
      public static final double compP = 20;
      public static final double compI = 0.0;
      public static final double compD = 0.0;
      public static final double compS = 10;

      public static final int indexerCanID = 25;

      public static final double rollerCurrentLimit = 110;
      public static final double rollerConversionFactor = 1;

      public static final double rollerMaxVelocity = 100;
      public static final double rollerMaxAcceleration = 200;

      public static final double statusUpdateFrequency = 20;
  }
  public static class Intake{
      public static final int rollerMotorCANID = 50;
      public static final double RollerCurrentLimit = 40;
      public static final double RollerGearRatio = 0.75;
      public static final double RollerMotionMagicCruise = 100;
      public static final double RollerMotionMagicAccel = 200;
      public static final double RollerkP = 0.6;
      public static final double RollerkI = 0.0;
      public static final double RollerkD = 0.0;
      public static final double RollerkS = 2.1;
      public static final double RollerkG = 0.0;

      public static final int pivotMotorCANID = 40;
      public static final double PivotCurrentLimit = 40;
      public static final double PivotGearRatio = 11.555;
      public static final double PivotMotionMagicCruise = 100;
      public static final double PivotMotionMagicAccel = 200;
      public static final double PivotkP = 10.0;
      public static final double PivotkI = 0.0;
      public static final double PivotkD = 0.0;
      public static final double PivotkS = 0.0;
      public static final double PivotkG = 0.0;

      public static final double FrequencyUpdateRate = 20;
      public static double rollerRadius = Units.inchesToMeters(1);
  }

}
