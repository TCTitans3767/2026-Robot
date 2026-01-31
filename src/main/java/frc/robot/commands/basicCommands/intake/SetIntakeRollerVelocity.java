package frc.robot.commands.basicCommands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetIntakeRollerVelocity extends Command {

    public final double velocity;

    public SetIntakeRollerVelocity(double velocity) {
        this.velocity = velocity;
    }

    @Override
    public void initialize() {
        Robot.intake.setRollerVelocity(this.velocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
