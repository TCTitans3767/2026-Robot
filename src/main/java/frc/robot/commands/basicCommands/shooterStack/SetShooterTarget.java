package frc.robot.commands.basicCommands.shooterStack;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetShooterTarget extends Command {

    private final Translation2d shooterTarget;

    public SetShooterTarget(Translation2d shooterTarget) {
        this.shooterTarget = shooterTarget;
    }

    @Override
    public void initialize() {
        Robot.shooterArray.setTarget(this.shooterTarget);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
