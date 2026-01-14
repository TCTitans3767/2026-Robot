package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    public static class HoodIOInputs {
        public double currentAngle;
        public double targetAngle;
        public double currentAmperage;
        public double currentTorque;
    }

    public default void updateInputs(HoodIOInputs inputs) {};

    public default void setAngle(double angle) {};

    public default void setPower(double percentage) {};
}
