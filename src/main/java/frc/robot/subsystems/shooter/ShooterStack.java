package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.shooter.feeder.Feeder;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class ShooterStack {
    private final Drivetrain drivetrain = Robot.drivetrain;
    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private final Feeder feeder;
    private final Pose2d robotRelativeOffset;
    private final String name;

    private boolean shootingEnabled = true;

    private Translation2d shotTarget = new Translation2d();
    private double distanceToTarget = 0;
    private InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

    private Pose2d currentShooterPosition = new Pose2d();

    private double idleVelocity = 2000;

    private int simCycleCount = 0;

    public ShooterStack(String name, Turret turret, Hood hood, Flywheel flywheel, Feeder feeder, Pose2d robotRelativeOffset) {
        this.robotRelativeOffset = robotRelativeOffset;
        this.turret = turret;
        this.flywheel = flywheel;
        this.hood = hood;
        this.feeder = feeder;
        this.name = name;
    }

    public void periodic() {
        currentShooterPosition = drivetrain.getPose().plus(new Transform2d(robotRelativeOffset.getTranslation(), robotRelativeOffset.getRotation()));
        Logger.recordOutput(name+" Turret Position", new Pose2d(currentShooterPosition.getX(), currentShooterPosition.getY(), Rotation2d.fromRadians(turret.getRotationFieldCoordinates())));
        distanceToTarget = currentShooterPosition.getTranslation().getDistance(shotTarget);
        Logger.recordOutput(name + " Distance To Target", distanceToTarget);
        double targetTurretRotation = calculateTurretRotation();

        turret.setRotation(targetTurretRotation + (0.1 * calculateVelocityAwayFromTarget(targetTurretRotation - drivetrain.getRotation().getRadians())));
        hood.setAngle(hoodMap.get(distanceToTarget) != null ? hoodMap.get(distanceToTarget) : 0);

        if (shootingEnabled) {
            flywheel.setVelocity(
                    flywheelMap.get(distanceToTarget) != null ?
                    flywheelMap.get(distanceToTarget) + Units.radiansToRotations(calculateFlywheelVelocityCorrection(targetTurretRotation - drivetrain.getRotation().getRadians()))
                    : 0
            );
        } else {
            flywheel.setVelocity(idleVelocity);
        }

        if (!(flywheel.getVelocity() >= flywheel.getTargetVelocity()) || !shootingEnabled) {
            feeder.setFeedSpeed(0);
        } else if ((flywheel.getVelocity() >= flywheel.getTargetVelocity()) && shootingEnabled) {
            feeder.setFeedVelocity(30);
        } else {
            feeder.setFeedVelocity(0);
        }

        if (Constants.currentMode == Constants.Mode.SIM) {
            if (simCycleCount >= 30) {
                shootSimFuel();
                simCycleCount = 0;
            } else {
                simCycleCount++;
            }
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
        return drivetrain.getRotation().getRadians() + Math.atan2(
                shotTarget.getY() - currentShooterPosition.getY(),
                shotTarget.getX() - currentShooterPosition.getX()
                );
//        drivetrain.getRotation().getRadians() -
    }

    private double calculateFlywheelVelocityCorrection(double angleToTarget) {
        double xVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getChassisSpeeds(), drivetrain.getRotation()).vxMetersPerSecond;
        double yVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getChassisSpeeds(), drivetrain.getRotation()).vyMetersPerSecond;

        return ((Math.sqrt((xVelocity * xVelocity) + (yVelocity * yVelocity)) * Math.cos(angleToTarget - Math.atan2(yVelocity, xVelocity))) / Constants.Shooter.flywheelDiameter);
    }

    private double calculateVelocityAwayFromTarget(double angleToTarget) {
        double xVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getChassisSpeeds(), drivetrain.getRotation()).vxMetersPerSecond;
        double yVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getChassisSpeeds(), drivetrain.getRotation()).vyMetersPerSecond;

        return (Math.sqrt((xVelocity * xVelocity) + (yVelocity * yVelocity)) * Math.cos(angleToTarget - Math.atan2(yVelocity, xVelocity)));
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

    private void shootSimFuel() {
        SimulatedArena.getInstance().addGamePieceProjectile(new RebuiltFuelOnFly(
                this.currentShooterPosition.getTranslation(),
                new Translation2d(),
                ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getChassisSpeeds(), drivetrain.getRotation()),
                Rotation2d.fromRadians(turret.getRotationFieldCoordinates()),
                Inches.of(20),
                MetersPerSecond.of(Units.rotationsToRadians(flywheel.getVelocity()) * Constants.Shooter.flywheelDiameter * 0.3),
                Radians.of(hood.getAngle())
        ));
    }
}
