package frc.robot.commands.basicCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.turret.Turret;
import org.littletonrobotics.junction.Logger;

public class HomeTurretCommand extends Command {

    private final Turret turret;

    private double previousEncoderReading;
    private double newEncoderReading;

    private int cyclesWithSameEncoderReading = 0;

    private boolean homingComplete = false;

    public HomeTurretCommand(Turret turret) {
        this.turret = turret;
    }

    @Override
    public void initialize() {
        previousEncoderReading = turret.getRotation();
        turret.setPower(-0.2);
    }

    @Override
    public void execute() {
        this.newEncoderReading = this.turret.getRotation();
        if (MathUtil.isNear(newEncoderReading, previousEncoderReading, 0.005) && DriverStation.isDSAttached() && DriverStation.isEnabled()) {
            cyclesWithSameEncoderReading++;
        }

        if (cyclesWithSameEncoderReading >= 20) {
            homingComplete = true;
        }

        this.previousEncoderReading = this.newEncoderReading;
    }

    @Override
    public boolean isFinished() {
        return homingComplete;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            System.out.println(turret.name + " has finished homing and is ready for the next step!");
            turret.resetEncoder(Constants.Shooter.Turret.leftLimit);
        } else {
            System.out.println("Something went wrong and the the homing sequence for {" + turret.name + "} quit unexpectedly");
        }
        turret.setPower(-0.005);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
