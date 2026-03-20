package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class PanicCommand extends Command {

    public PanicCommand() {
    }

    @Override
    public void initialize() {
        Robot.shooterArray.disableTargetPointing();
        Robot.shooterArray.setInterpolationMaps(new InterpolatingDoubleTreeMap(), new InterpolatingDoubleTreeMap());
    }

    @Override
    public void execute() {
        if (Robot.driverController.getLeftTriggerAxis() > 0.5) {
            // Intake mode
            Robot.shooterArray.enableShooting(false);
            Robot.intake.setRollerVelocity(60);
//      Robot.indexer.setIndexVelocity(25);
            Robot.intake.setPivotPosition(-0.5);
            Robot.shooterArray.setFeederVelocity(-10);
        } else if (Robot.driverController.getRightTriggerAxis() > 0.5) {
            // Shoot mode
            Robot.shooterArray.enableShooting(true);
            Robot.shooterArray.setHoodAngle(1100);
            Robot.intake.setRollerVelocity(45);
            Robot.indexer.setIndexVelocity(25);
            Robot.intake.setPivotPosition(0.25);
        } else if (Robot.driverController.getLeftBumperButton()) {
            Robot.intake.setRollerVelocity(-30);
            Robot.indexer.setIndexVelocity(-25);
            Robot.shooterArray.setFeederVelocity(-20);
        } else {
            // Neutral mode
            Robot.shooterArray.enableShooting(false);
            Robot.intake.setRollerSpeed(0);
            Robot.indexer.setIndexSpeed(0);
            Robot.shooterArray.setHoodAngle(1000);
            Robot.shooterArray.setFeederVelocity(0);
            Robot.intake.setPivotSpeed(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
