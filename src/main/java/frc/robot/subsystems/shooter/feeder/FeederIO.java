package frc.robot.subsystems.shooter.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

    @AutoLog
    public static class FeederIOInputs {
        public double currentSpeed;
        public double currentAmperage;
        public double targetSpeed;
    }

    public default void updateInputs(FeederIOInputs inputs) {};

    public default void setFeedSpeed(double speed) {};
}
