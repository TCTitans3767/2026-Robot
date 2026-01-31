package frc.robot.commands.basicCommands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetIntakePivotSpeed extends Command {

    private final double speed;

    public SetIntakePivotSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void initialize() {
        Robot.intake.setPivotSpeed(this.speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
