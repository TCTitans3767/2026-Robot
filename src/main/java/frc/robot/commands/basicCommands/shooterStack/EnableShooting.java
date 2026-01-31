package frc.robot.commands.basicCommands.shooterStack;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class EnableShooting extends Command {

    public EnableShooting() {}

    @Override
    public void initialize() {
        Robot.shooterArray.enableShooting(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
