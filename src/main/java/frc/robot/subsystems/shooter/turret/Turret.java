package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretIO io;
    private final String name;

    public Turret(String name, TurretIO io) {
        this.io = io;
        this.name = name;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    public void setRotation(double targetRotation) {
//        Logger.recordOutput(name+"rotation Target", targetRotation);
        io.setRotation(targetRotation);
    }

    public void setPower(double percentage) {
        io.setPower(percentage);
    }

    public double getRotation() {
        return inputs.currentRotation;
    }
}
