package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    public static class FlywheelIOInputs {
        public double targetVelocity;
        public double currentVelocity;
        public double currentTorque;
        public double currentAmperage;
    }

    public default void updateInputs(FlywheelIOInputs inputs) {};

    public default void setVelocity(double radiansPerSecond) {};

    public default void setPower(double percent) {};
}
