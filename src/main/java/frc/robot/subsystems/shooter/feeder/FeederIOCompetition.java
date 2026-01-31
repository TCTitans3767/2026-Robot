package frc.robot.subsystems.shooter.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;

public class FeederIOCompetition implements FeederIO{

    private enum ControlMode {
        POWER,
        VELOCITY
    }

    private final TalonFX feederMotor;
    private final TalonFXConfiguration feederMotorConfig = new TalonFXConfiguration();
    private final Slot0Configs slot0Config = new Slot0Configs();
    private final MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();

    private double targetPower = 0;
    private double targetVelocity = 0;
    private ControlMode controlMode = ControlMode.POWER;

    private final StatusSignal<AngularVelocity> velocityStatusSignal;
    private final StatusSignal<Current> amperageStatusSignal;
    private final StatusSignal<Current> torqueStatusSignal;

    public FeederIOCompetition(int CANID) {
        this.feederMotor = new TalonFX(CANID);

        feederMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        feederMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feederMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feederMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.Feeder.currentLimit;
        feederMotorConfig.Feedback.RotorToSensorRatio = Constants.Shooter.Feeder.gearRatio;

        slot0Config.kP = Constants.Shooter.Feeder.compP;
        slot0Config.kI = Constants.Shooter.Feeder.compI;
        slot0Config.kD = Constants.Shooter.Feeder.compD;
        slot0Config.kS = Constants.Shooter.Feeder.compS;

        motionMagicConfig.MotionMagicAcceleration = Constants.Shooter.Feeder.motionMagicAccel;
        motionMagicConfig.MotionMagicCruiseVelocity = Constants.Shooter.Feeder.motionMagicCruise;

        velocityStatusSignal = feederMotor.getVelocity();
        amperageStatusSignal = feederMotor.getStatorCurrent();
        torqueStatusSignal = feederMotor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(Constants.Shooter.signalUpdateFrequency, velocityStatusSignal, amperageStatusSignal, torqueStatusSignal);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        switch (this.controlMode) {
            case POWER -> this.feederMotor.set(targetPower);
            case VELOCITY -> this.feederMotor.setControl(new MotionMagicVelocityVoltage(targetVelocity));
        }

        inputs.currentAmperage = amperageStatusSignal.getValueAsDouble();
        inputs.currentTorque = torqueStatusSignal.getValueAsDouble();
        inputs.currentVelocity = velocityStatusSignal.getValueAsDouble();
        inputs.targetVelocity = targetVelocity;
    }

    @Override
    public void setVelocity(double velocity) {
        this.controlMode = ControlMode.VELOCITY;
        this.targetVelocity = velocity;
    }

    @Override
    public void setFeedSpeed(double speed) {
        this.controlMode = ControlMode.POWER;
        this.targetPower = speed;
    }
}
