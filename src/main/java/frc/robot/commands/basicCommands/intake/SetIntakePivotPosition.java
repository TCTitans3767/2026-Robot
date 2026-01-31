package frc.robot.commands.basicCommands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetIntakePivotPosition extends Command {

    private final double position;

    public SetIntakePivotPosition(double position) {
        this.position = position;
    }

    @Override
    public void initialize() {
        Robot.intake.setPivotPosition(this.position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
