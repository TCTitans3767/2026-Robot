package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class BasicAuton extends SequentialCommandGroup {

    public BasicAuton() {
        addCommands(
                new InstantCommand(() -> Robot.drivetrain.setPose(new Pose2d(new Translation2d(), new Rotation2d(Units.degreesToRadians(180))))),
                new InstantCommand(() -> {
                    Robot.intake.setPivotPosition(0);
                    Robot.intake.setRollerVelocity(20);
                    Robot.indexer.setIndexVelocity(20);
                    Robot.shooterArray.enableShooting(true);
                })
        );
    }
}
