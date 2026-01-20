package frc.robot.subsystems.shooter.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
    private final FeederIO io;
    private final String name;

    public Feeder(String name, FeederIO io) {
        this.io = io;
        this.name = name;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    public void setFeedSpeed(double speed) {
        io.setFeedSpeed(speed);
    }

    public void setFeedVelocity(double velocity) {io.setVelocity(velocity);}
}
