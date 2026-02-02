package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final IndexerIO io;

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    public void setIndexSpeed(double speed) {
        io.setIndexSpeed(speed);
    }

    public void setIndexVelocity(double velocity) {io.setVelocity(velocity);}
}
