package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.drive.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class TriggerBoard {

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
}
