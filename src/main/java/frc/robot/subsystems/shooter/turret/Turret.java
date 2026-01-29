package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretIO io;
    public final String name;

    public Turret(String name, TurretIO io) {
        this.io = io;
        this.name = name;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    public void setRotation(double targetRotationRads) {
//        Logger.recordOutput(name+"rotation Target", targetRotation);
        io.setRotation(targetRotationRads);
    }

    public void setPower(double percentage) {
        io.setPower(percentage);
    }

    public double getRotation() {
        return inputs.currentRotation;
    }

    public double getRotationFieldCoordinates() {
        return inputs.currentRotation - Robot.drivetrain.getRotation().getRadians();
    }

    public double getVelocity() {
        return inputs.currentVelocity;
    }

    public void resetEncoder(double degrees) {
        io.resetEncoder(Units.degreesToRotations(degrees));
    }
}
