package frc.robot.subsystems.shooter.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

    @AutoLog
    public static class FeederIOInputs {
        public double currentVelocity;
        public double currentAmperage;
        public double currentTorque;
        public double targetVelocity;
    }

    public default void updateInputs(FeederIOInputs inputs) {};

    public default void setFeedSpeed(double speed) {};

    public default void setVelocity(double velocity) {};
}
