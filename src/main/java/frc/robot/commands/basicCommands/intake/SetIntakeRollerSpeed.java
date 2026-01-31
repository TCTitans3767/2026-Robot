package frc.robot.commands.basicCommands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetIntakeRollerSpeed extends Command {

    private final double speed;

    public SetIntakeRollerSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void initialize() {
        Robot.intake.setRollerSpeed(this.speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
