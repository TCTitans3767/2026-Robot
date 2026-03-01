package frc.robot.commands.basicCommands.shooterStack;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetFlywheelPower extends Command {

    private final double targetPower;

    public SetFlywheelPower(double power) {
        this.targetPower = power;
    }

    @Override
    public void initialize() {
        Robot.shooterArray.setFlywheelPower(this.targetPower);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
