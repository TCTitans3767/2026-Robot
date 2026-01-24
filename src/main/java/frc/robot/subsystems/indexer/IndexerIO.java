package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        public double currentVelocity;
        public double currentAmperage;
        public double currentTorque;
        public double targetVelocity;
    }

    public default void updateInputs(IndexerIOInputs inputs) {};

    public default void setIndexSpeed(double speed) {};

    public default void setVelocity(double velocity) {};
}
