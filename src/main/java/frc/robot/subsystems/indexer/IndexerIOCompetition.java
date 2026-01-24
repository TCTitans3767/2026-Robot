package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.Constants;
import jdk.jshell.Snippet;

public class IndexerIOCompetition implements IndexerIO{

    private final TalonFX indexerMotor = new TalonFX(Constants.Indexer.indexerCanID);
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    private final Slot0Configs slot0Configs = new Slot0Configs();

    private double velocityTarget = 0;

    private final StatusSignal<AngularVelocity> velocityStatusSignal;
    private final StatusSignal<Current> amperageStatusSignal;
    private final StatusSignal<Current> torqueStatusSignal;

    // Motor Setup

    public IndexerIOCompetition() {
        config.CurrentLimits.StatorCurrentLimit = Constants.Indexer.rollerCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = Constants.Indexer.rollerConversionFactor;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Indexer.rollerMaxVelocity;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Indexer.rollerMaxAcceleration;

        slot0Configs.kP = Constants.Indexer.compP;
        slot0Configs.kI = Constants.Indexer.compI;
        slot0Configs.kD = Constants.Indexer.compD;
        slot0Configs.kS = Constants.Indexer.compS;

        indexerMotor.getConfigurator().apply(config);
        indexerMotor.getConfigurator().apply(slot0Configs);
        indexerMotor.getConfigurator().apply(motionMagicConfigs);
        indexerMotor.setNeutralMode(NeutralModeValue.Coast);

        velocityStatusSignal = indexerMotor.getVelocity();
        amperageStatusSignal = indexerMotor.getStatorCurrent();
        torqueStatusSignal = indexerMotor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(Constants.Indexer.statusUpdateFrequency, amperageStatusSignal, torqueStatusSignal, velocityStatusSignal);
    }

    @Override
    public void updateInputs(IndexerIO.IndexerIOInputs inputs) {
        indexerMotor.setControl(new MotionMagicVoltage(velocityTarget));

        inputs.targetVelocity = this.velocityTarget;
        inputs.currentAmperage = amperageStatusSignal.getValueAsDouble();
        inputs.currentTorque = torqueStatusSignal.getValueAsDouble();
        inputs.currentVelocity = velocityStatusSignal.getValueAsDouble();
    }

    @Override
    public void setVelocity(double rotationsPerSecond) {
        this.velocityTarget = rotationsPerSecond;
    }
}
