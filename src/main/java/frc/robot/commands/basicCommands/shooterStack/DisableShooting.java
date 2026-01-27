package frc.robot.commands.basicCommands.shooterStack;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DisableShooting extends Command {

    public DisableShooting() {};

    @Override
    public void initialize() {
        Robot.shooterArray.enableShooting(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
