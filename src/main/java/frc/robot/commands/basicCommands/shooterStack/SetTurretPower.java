package frc.robot.commands.basicCommands.shooterStack;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

public class SetTurretPower extends Command {
    private final DoubleSupplier targetPower;

    public SetTurretPower(DoubleSupplier power) {
        this.targetPower = power;
    }

    @Override
    public void execute() {
        Robot.shooterArray.setTurretPower(targetPower.getAsDouble());
    }
}
