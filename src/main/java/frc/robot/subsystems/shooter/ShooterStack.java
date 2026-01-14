package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.shooter.feeder.Feeder;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterStack extends SubsystemBase {
    private final Drivetrain drivetrain = Robot.drivetrain;
    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private final Feeder feeder;
    private final Pose2d robotRelativeOffset;
    private final String name;

    private boolean shootingEnabled;

    private Translation2d shotTarget = new Translation2d();
    private double distanceToTarget = 0;
    private InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

    private Pose2d currentShooterPosition = new Pose2d();

    private double idleVelocity = 2000;

    public ShooterStack(String name, Turret turret, Hood hood, Flywheel flywheel, Feeder feeder, Pose2d robotRelativeOffset) {
        this.robotRelativeOffset = robotRelativeOffset;
        this.turret = turret;
        this.flywheel = flywheel;
        this.hood = hood;
        this.feeder = feeder;
        this.name = name;
    }

    @Override
    public void periodic() {
        currentShooterPosition = drivetrain.getPose().plus(new Transform2d(robotRelativeOffset.getTranslation(), robotRelativeOffset.getRotation()));
//        Logger.recordOutput(name+"position", currentShooterPosition);
        distanceToTarget = currentShooterPosition.getTranslation().getDistance(shotTarget);

        turret.setRotation(calculateTurretRotation());
        hood.setAngle(hoodMap.get(distanceToTarget) != null ? hoodMap.get(distanceToTarget) : 0);

        if (shootingEnabled) {
            flywheel.setVelocity(flywheelMap.get(distanceToTarget) != null ? flywheelMap.get(distanceToTarget) : 0);
        } else {
            flywheel.setVelocity(idleVelocity);
        }

        if (!(flywheel.getVelocity() >= flywheel.getTargetVelocity()) || !shootingEnabled) {
            feeder.setFeedSpeed(0);
        } else if ((flywheel.getVelocity() >= flywheel.getTargetVelocity()) && shootingEnabled) {
            feeder.setFeedSpeed(0.5);
        } else {
            feeder.setFeedSpeed(0);
        }
    }

    public void setTarget(Translation2d target) {
        this.shotTarget = target;
    }

    public void setInterpolationMaps(InterpolatingDoubleTreeMap hoodMap, InterpolatingDoubleTreeMap flywheelMap) {
        this.hoodMap = hoodMap;
        this.flywheelMap = flywheelMap;
    }

    public Pose2d getShooterPosition() {
        return currentShooterPosition;
    }

    private double calculateTurretRotation() {
        return drivetrain.getRotation().getRadians() - Math.atan2(
                shotTarget.getY() - currentShooterPosition.getY(),
                shotTarget.getX() - currentShooterPosition.getX()
                );
    }

    public void setTurretAngle(double angle) {
        turret.setRotation(angle);
    }

    public void setHoodAngle(double angle) {
        hood.setAngle(angle);
    }

    public void setFlywheelSpeed(double velocity) {
        flywheel.setVelocity(velocity);
    }

    public void enableShooting(boolean shootingEnabled) {
        this.shootingEnabled = shootingEnabled;
    }

    public String getShooterName() {
        return name;
    }
}
