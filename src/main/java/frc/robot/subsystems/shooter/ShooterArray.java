package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class ShooterArray extends SubsystemBase{

    Map<String, ShooterStack> shooterStacks = new HashMap<>();

    public ShooterArray() {

    }

    @Override
    public void periodic() {
        shooterStacks.forEach(((shooterStackName, shooterStack) -> {
            shooterStack.periodic();
        }));
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

    public void disableTragetPointing() {
        shooterStacks.forEach((shooterStackName, shooterStack) -> {
            shooterStack.disablePointToTarget();
        });
    }
    public void enableTragetPointing() {
        shooterStacks.forEach((shooterStackName, shooterStack) -> {
            shooterStack.enablePointToTarget();
        });
    }

    public Command homeTurrets() {
        ParallelCommandGroup homeAllTurretsCommand = new ParallelCommandGroup() {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };

        shooterStacks.forEach((shooterStackName, shooterStack) -> {
            homeAllTurretsCommand.addCommands(shooterStack.homeTurretCommand());
        });

        return homeAllTurretsCommand;
    }

    public void setFeederPower(double power) {
        shooterStacks.forEach((shooterStackName, shooterStack) -> {
            shooterStack.setFeederPower(power);
        });
    }

    public void setTurretPower(double power) {
        shooterStacks.forEach((shooterStackName, shooterStack) -> {
            shooterStack.setTurretPower(power);
        });
    }

    public void setFlywheelPower(double power) {
        shooterStacks.forEach((shooterStackName, shooterStack) -> {
            shooterStack.setFlywheelPower(power);
        });
    }

    public void setFlywheelVelocity(double velocity) {
        shooterStacks.forEach((shooterStackName, shooterStack) -> {
            shooterStack.setFlywheelSpeed(velocity);
        });
    }

    public void setFeederVelocity(double velocity) {
        shooterStacks.forEach((shooterStackName, shooterStack) -> {
            shooterStack.setFeederPower(-10);
        });
    }

    public void setHoodAngle(int angle) {
        shooterStacks.forEach((shooterStackName, shooterStack) -> {
            shooterStack.setHoodAngle(angle);
        });
    }
}
