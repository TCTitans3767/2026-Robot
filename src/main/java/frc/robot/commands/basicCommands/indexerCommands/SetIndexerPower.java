package frc.robot.commands.basicCommands.indexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetIndexerPower extends Command {

    private final double targetPower;

    public SetIndexerPower(double targetPower) {
        this.targetPower = targetPower;
    }

    @Override
    public void initialize() {
        Robot.indexer.setIndexSpeed(this.targetPower);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
