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

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.subsystems.shooter.ShooterArray;
import frc.robot.subsystems.shooter.ShooterStack;
import frc.robot.subsystems.shooter.feeder.Feeder;
import frc.robot.subsystems.shooter.feeder.FeederIO;
import frc.robot.subsystems.shooter.feeder.FeederIOSim;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.utils.DriveModes;
import frc.robot.utils.RobotStates;
import frc.robot.utils.RobotTransitions;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  public static Drivetrain drivetrain;
  public static ShooterArray shooterArray = new ShooterArray();

  public static SwerveDriveSimulation driveSimulation = null;

  public static final XboxController driverController = new XboxController(Constants.DriverController.PORT);

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    switch (Constants.currentMode) {
          case REAL:
              // Real robot, instantiate hardware IO implementations
              drivetrain =
                      new Drivetrain(
                              new GyroIOPigeon2(),
                              new ModuleIOReal(TunerConstants.FrontLeft),
                              new ModuleIOReal(TunerConstants.FrontRight),
                              new ModuleIOReal(TunerConstants.BackLeft),
                              new ModuleIOReal(TunerConstants.BackRight));
              shooterArray.addShooter(new ShooterStack(
                      "LeftShooterStack",
                      new Turret("leftTurret", new TurretIO() {}),
                      new Hood("leftHood", new HoodIO() {}),
                      new Flywheel("leftFlywheel", new FlywheelIO() {}),
                      new Feeder("leftFeeder", new FeederIO() {}),
                      new Pose2d()
              ));
              shooterArray.addShooter(new ShooterStack(
                      "RightShooterStack",
                      new Turret("rightTurret", new TurretIO() {}),
                      new Hood("rightHood", new HoodIO() {}),
                      new Flywheel("rightFlywheel", new FlywheelIO() {}),
                      new Feeder("rightFeeder", new FeederIO() {}),
                      new Pose2d()
              ));
              break;

          case SIM:
              // Sim robot, instantiate physics sim IO implementations

            driveSimulation = new SwerveDriveSimulation(Drivetrain.mapleSimConfig, new Pose2d(2, 2, new Rotation2d()));
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
              drivetrain =
                      new Drivetrain(
                              new GyroIO() {},
                              new ModuleIOSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                              new ModuleIOSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                              new ModuleIOSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                              new ModuleIOSim(TunerConstants.BackRight, driveSimulation.getModules()[3]));
              shooterArray.addShooter(new ShooterStack(
                      "LeftShooterStack",
                      new Turret("leftTurret", new TurretIOSim()),
                      new Hood("leftHood", new HoodIOSim()),
                      new Flywheel("leftFlywheel", new FlywheelIOSim()),
                      new Feeder("leftFeeder", new FeederIOSim()),
                      new Pose2d(Units.inchesToMeters(-8), Units.inchesToMeters(8), new Rotation2d())
              ));
              shooterArray.addShooter(new ShooterStack(
                      "RightShooterStack",
                      new Turret("rightTurret", new TurretIOSim()),
                      new Hood("rightHood", new HoodIOSim()),
                      new Flywheel("rightFlywheel", new FlywheelIOSim()),
                      new Feeder("rightFeeder", new FeederIOSim()),
                      new Pose2d(Units.inchesToMeters(-8), Units.inchesToMeters(-8), new Rotation2d())
              ));

              shooterArray.setTarget(Constants.FieldPoses.blueHub);
              shooterArray.setInterpolationMaps(Constants.Shooter.simHoodAngleInterpolationMap, Constants.Shooter.simFlywheelVelocityInterpolationMap);
              break;

          default:
              // Replayed robot, disable IO implementations
              drivetrain =
                      new Drivetrain(
                              new GyroIO() {},
                              new ModuleIO() {},
                              new ModuleIO() {},
                              new ModuleIO() {},
                              new ModuleIO() {});
              shooterArray.addShooter(new ShooterStack(
                      "LeftShooterStack",
                      new Turret("leftTurret", new TurretIO() {}),
                      new Hood("leftHood", new HoodIO() {}),
                      new Flywheel("leftFlywheel", new FlywheelIO() {}),
                      new Feeder("leftFeeder", new FeederIO() {}),
                      new Pose2d()
              ));
              shooterArray.addShooter(new ShooterStack(
                      "RightShooterStack",
                      new Turret("rightTurret", new TurretIO() {}),
                      new Hood("rightHood", new HoodIO() {}),
                      new Flywheel("rightFlywheel", new FlywheelIO() {}),
                      new Feeder("rightFeeder", new FeederIO() {}),
                      new Pose2d()
              ));
              break;
    }

    new RobotControl();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

      RobotStates.initStates();
      RobotTransitions.initTransitions();
      DriveModes.initDriveModes();

    // initialize default state and drive commands
    RobotControl.setDriveModeCommand(DriveModes.teleopDrive);
//    drivetrain.setPose(new Pose2d(0, 2, Rotation2d.fromDegrees(32)));
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
//    driveSimulation.setAngularVelocity(Units.degreesToRadians(15));

    if (drivetrain.getPose().getY() < 7.3) {
      driveSimulation.setLinearVelocity(0, 0.5);
    }
  }

    public static DriverStation.Alliance getAlliance() {
      return DriverStation.isDSAttached() ? DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) : DriverStation.Alliance.Blue;
    }
}
