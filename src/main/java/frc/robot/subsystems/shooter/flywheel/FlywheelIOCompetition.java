package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;


public class FlywheelIOCompetition implements FlywheelIO{

    private enum ControlMode {
        POWER,
        VELOCITY
    }

    private final TalonFX flywheelMotor;
    private final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    private final Slot0Configs flywheelSlot0Config = new Slot0Configs();
    private final MotionMagicConfigs flywheelMotionMagicConfig = new MotionMagicConfigs();

    private double targetVelocity = 0;
    private double targetPower = 0;
    private ControlMode controlMode = ControlMode.POWER;

    private final StatusSignal<AngularVelocity> velocityStatusSignal;
    private final StatusSignal<Current> amperageStatusSignal;
    private final StatusSignal<Current> torqueStatusSignal;

    public FlywheelIOCompetition(int CANID) {
        flywheelMotor = new TalonFX(CANID);

        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.Feedback.RotorToSensorRatio = Constants.Shooter.Flywheel.gearRatio;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.Flywheel.currentLimit;

        flywheelSlot0Config.kP = Constants.Shooter.Flywheel.compP;
        flywheelSlot0Config.kI = Constants.Shooter.Flywheel.compI;
        flywheelSlot0Config.kD = Constants.Shooter.Flywheel.compD;
        flywheelSlot0Config.kS = Constants.Shooter.Flywheel.compS;

        flywheelMotionMagicConfig.MotionMagicCruiseVelocity = Constants.Shooter.Flywheel.motionMagicCruise;
        flywheelMotionMagicConfig.MotionMagicAcceleration = Constants.Shooter.Flywheel.motionMagicAccel;

        flywheelMotor.getConfigurator().apply(flywheelConfig);
        flywheelMotor.getConfigurator().apply(flywheelSlot0Config);
        flywheelMotor.getConfigurator().apply(flywheelMotionMagicConfig);

        velocityStatusSignal = flywheelMotor.getVelocity();
        amperageStatusSignal = flywheelMotor.getStatorCurrent();
        torqueStatusSignal = flywheelMotor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(Constants.Shooter.signalUpdateFrequency, velocityStatusSignal, amperageStatusSignal, torqueStatusSignal);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        switch (controlMode) {
            case POWER -> flywheelMotor.set(targetPower);
            case VELOCITY -> flywheelMotor.setControl(new MotionMagicVelocityTorqueCurrentFOC(targetVelocity));
        }

        inputs.targetVelocity = this.targetVelocity;
        inputs.currentVelocityRPS = velocityStatusSignal.getValueAsDouble();
        inputs.currentVelocityRPM = velocityStatusSignal.getValueAsDouble() * 60;
        inputs.currentTorque = torqueStatusSignal.getValueAsDouble();
        inputs.currentAmperage = amperageStatusSignal.getValueAsDouble();
    }

    @Override
    public void setVelocity(double rotationsPerSecond) {
        this.controlMode = ControlMode.VELOCITY;
        this.targetVelocity = rotationsPerSecond;
    }

    @Override
    public void setPower(double percent) {
        this.controlMode = ControlMode.POWER;
        this.targetPower = percent;
    }
}
