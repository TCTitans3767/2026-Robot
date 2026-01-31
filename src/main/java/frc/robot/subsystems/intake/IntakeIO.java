package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double currentRollerVelocity = 0.0;
        public double currentRollerAmperage = 0.0;
        public double currentRollerTorque = 0.0;
        public double targetRollerVelocity = 0.0;
        public double targetRollerPower = 0.0;

        public double currentPivotVelocity = 0.0;
        public double currentPivotPosition = 0.0;
        public double currentPivotAmperage = 0.0;
        public double currentPivotTorque = 0.0;
        public double targetPivotVelocity = 0.0;
        public double targetPivotPosition = 0.0;
        public double targetPivotPower = 0.0;
    }

    public default void updateInputs(IntakeIO.IntakeIOInputs inputs) {};

    public default void setPivotSpeed(double speed) {};
    public default void setPivotPosition(double position) {};

    public default void setRollerSpeed(double velocity) {};
    public default void setRollerVelocity(double velocity) {};

}
