package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.HubState;

import java.util.List;

import static edu.wpi.first.units.Units.Seconds;

public class TriggerBoard {

    private static boolean autonomousIntake = false;
    private static boolean autonomousShoot = false;

    public static void setAutonomousIntake(boolean value) {
        autonomousIntake = value;
    }

    public static void setAutonomousShoot(boolean value) {
        autonomousShoot = value;
    }

    public static Command enableIntakeButtonAutonomous() {
        return new InstantCommand(() -> {autonomousIntake = true;});
    }
    public static Command disableIntakeButtonAutonomous() {
        return new InstantCommand(() -> {autonomousIntake = false;});
    }
    public static Command enableShootingButtonAutonomous() {
        return new InstantCommand(() -> {autonomousShoot = true;});
    }
    public static Command disableShootingButtonAutonomous() {
        return new InstantCommand(() -> {autonomousShoot = false;});
    }

    public static boolean isInNeutralZone() {
        return (Robot.drivetrain.getPose().getX() > 4.44) && (Robot.drivetrain.getPose().getX() < 12.1);
    }

    public static boolean isOnSide() {
        return Robot.getAlliance() == DriverStation.Alliance.Blue ? (Robot.drivetrain.getPose().getX() < 4.44) : (Robot.drivetrain.getPose().getX() > 12.128);
    }

    public static boolean isOffSide() {
        return Robot.getAlliance() == DriverStation.Alliance.Red ? (Robot.drivetrain.getPose().getX() < 4.44) : Robot.drivetrain.getPose().getX() > 12.128;
    }

    public static boolean leftOfHub() {
        if (Robot.getAlliance() == DriverStation.Alliance.Blue) {
            return Robot.drivetrain.getPose().getY() > Constants.FieldPoses.blueHub.getY();
        } else {
            return Robot.drivetrain.getPose().getY() < Constants.FieldPoses.redHub.getY();
        }
    }

    public static boolean isHopperFull() {
        return true;
    }

    public static boolean isRobotInNoShootingZone() {
        boolean isInNoShootZone = false;
        for (List<Translation2d> noShootZone : Constants.FieldPoses.noShootZones) {
            isInNoShootZone = isInNoShootZone | Robot.drivetrain.isInsideRectangle(noShootZone.get(0), noShootZone.get(1));
       }
        return isInNoShootZone;
    }

    public static boolean isShiftSoon() {
       return HubState.timeRemainingInCurrentShift().get().in(Seconds) <= Constants.shiftOffset;
    }

    public static boolean isHubActive() {
        return Robot.hubStateButton.get();
    }

    public static boolean isShootButtonPressed() {
        if (DriverStation.isDSAttached() && DriverStation.isAutonomousEnabled()) {
            return autonomousShoot;
        } else {
            return Robot.driverController.getRightTriggerAxis() > 0.5;
        }
    }

    public static boolean isIntakeButtonPressed() {
        if (DriverStation.isDSAttached() && DriverStation.isAutonomousEnabled()) {
            return autonomousIntake;
        } else {
            return Robot.driverController.getLeftTriggerAxis() > 0.5;
        }
    }
}
