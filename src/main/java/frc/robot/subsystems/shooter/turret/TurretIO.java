package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    public static class TurretIOInputs {
        public double targetRotation = 0;
        public boolean atLeftLimit = false;
        public boolean atRightLimit = false;
        public double currentRotation = 0;
        public double currentVelocity = 0;
        public double currentAmperage;
        public double currentTorque;
    }

    public default void updateInputs(TurretIOInputs inputs) {};

    public default void setRotation(double targetRotation) {};

    public default void setPower(double percentage) {};
}
