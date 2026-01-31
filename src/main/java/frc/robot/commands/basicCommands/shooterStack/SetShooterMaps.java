package frc.robot.commands.basicCommands.shooterStack;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Robot;

import java.util.Map;
import java.util.function.BooleanSupplier;

public class SetShooterMaps extends Command {

    private final InterpolatingDoubleTreeMap hoodMap;
    private final InterpolatingDoubleTreeMap flywheelMap;

    public SetShooterMaps(InterpolatingDoubleTreeMap hoodMap, InterpolatingDoubleTreeMap flywheelMap) {
        this.hoodMap = hoodMap;
        this.flywheelMap = flywheelMap;
    }

    @Override
    public void initialize() {
        Robot.shooterArray.setInterpolationMaps(hoodMap, flywheelMap);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
