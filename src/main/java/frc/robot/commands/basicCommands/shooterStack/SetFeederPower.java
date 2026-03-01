package frc.robot.commands.basicCommands.shooterStack;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetFeederPower extends Command {
    private final double targetPower;

    public SetFeederPower(double power) {
        this.targetPower = power;
    }

    @Override
    public void initialize() {
        Robot.shooterArray.setFeederPower(this.targetPower);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
