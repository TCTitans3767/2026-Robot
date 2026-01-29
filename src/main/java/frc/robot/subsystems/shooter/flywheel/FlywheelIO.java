package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    public static class FlywheelIOInputs {
        public double targetVelocity;
        public double currentVelocityRPS;
        public double currentTorque;
        public double currentAmperage;
        public double currentVelocityRPM;
    }

    public default void updateInputs(FlywheelIOInputs inputs) {};

    public default void setVelocity(double rotationsPerSecond) {};

    public default void setPower(double percent) {};
}
