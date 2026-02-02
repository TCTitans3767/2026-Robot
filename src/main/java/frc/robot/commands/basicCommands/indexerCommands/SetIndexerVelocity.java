package frc.robot.commands.basicCommands.indexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetIndexerVelocity extends Command {

    private final double targetVelocity;

    public SetIndexerVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    @Override
    public void initialize() {
        Robot.indexer.setIndexVelocity(this.targetVelocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
