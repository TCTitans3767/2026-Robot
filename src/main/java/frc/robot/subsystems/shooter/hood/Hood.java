package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final HoodIO io;
    private final String name;

    public Hood(String name, HoodIO io) {
        this.name = name;
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    public void setAngle(double angle) {
        io.setAngle(angle);
    }

    public void setPower(double percentage) {
        io.setPower(percentage);
    }

    public double getAngle() {
        return inputs.currentAngle;
    }
}
