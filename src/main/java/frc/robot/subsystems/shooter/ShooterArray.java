package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class ShooterArray extends SubsystemBase {

    Map<String, ShooterStack> shooterStacks = new HashMap<>();

    public ShooterArray() {

    }

    @Override
    public void periodic() {

    }

    public void addShooter(ShooterStack shooterStack) {
        shooterStacks.put(shooterStack.getShooterName(), shooterStack);
    }

    public void setTarget(Translation2d target) {
        shooterStacks.forEach(((shooterStackName, shooterStack) -> {
            shooterStack.setTarget(target);
        }));
    }

    public void setInterpolationMaps(InterpolatingDoubleTreeMap hoodMap, InterpolatingDoubleTreeMap flywheelMap) {
        shooterStacks.forEach((shooterStackName, shooterStack) -> {
            shooterStack.setInterpolationMaps(hoodMap, flywheelMap);
        });
    }

    public void enableShooting(boolean shootingEnabled) {
        shooterStacks.forEach((shooterStackName, shooterStack) -> {
            shooterStack.enableShooting(shootingEnabled);
        });
    }

}
