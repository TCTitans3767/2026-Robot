package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;

public class IntakeIOCompetition implements IntakeIO{

    private enum ControlMode {
        POWER,
        POSITION,
        VELOCITY
    }

    private final TalonFX rollerMotor, pivotMotor;
    private final TalonFXConfiguration rollerMotorConfiguration, pivotMotorConfiguration;
    private final Slot0Configs rollerMotorSlot0Config, pivotMotorSlot0Config;
    private final MotionMagicConfigs rollerMotorMotionMagicConfig, pivotMotorMotionMagicConfig;

    private ControlMode rollerControlMode, pivotControlMode;
    private double rollerTargetPower = 0;
    private double rollerTargetVelocity = 0;

    private double pivotTargetPower = 0;
    private double pivotTargetPosition = 0;

    private final StatusSignal<AngularVelocity> rollerMotorVelocity;
    private final StatusSignal<Current> rollerMotorAmperage;
    private final StatusSignal<Current> rollerMotorTorque;

    private final StatusSignal<Angle> pivotMotorPosition;
    private final StatusSignal<AngularVelocity> pivotMotorVelocity;
    private final StatusSignal<Current> pivotMotorAmperage;
    private final StatusSignal<Current> pivotMotorTorque;

    public IntakeIOCompetition() {
        this.rollerMotor = new TalonFX(Constants.Intake.rollerMotorCANID);
        this.rollerMotorConfiguration = new TalonFXConfiguration();
        this.rollerMotorSlot0Config = new Slot0Configs();
        this.rollerMotorMotionMagicConfig = new MotionMagicConfigs();

        this.pivotMotor = new TalonFX(Constants.Intake.pivotMotorCANID);
        this.pivotMotorConfiguration = new TalonFXConfiguration();
        this.pivotMotorSlot0Config = new Slot0Configs();
        this.pivotMotorMotionMagicConfig = new MotionMagicConfigs();

        rollerMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerMotorConfiguration.CurrentLimits.StatorCurrentLimit = Constants.Intake.RollerCurrentLimit;
        rollerMotorConfiguration.Feedback.RotorToSensorRatio = Constants.Intake.RollerGearRatio;

        rollerMotorMotionMagicConfig.MotionMagicCruiseVelocity = Constants.Intake.RollerMotionMagicCruise;
        rollerMotorMotionMagicConfig.MotionMagicAcceleration = Constants.Intake.RollerMotionMagicAccel;

        rollerMotorSlot0Config.GravityType = GravityTypeValue.Arm_Cosine;
        rollerMotorSlot0Config.kP = Constants.Intake.RollerkP;
        rollerMotorSlot0Config.kI = Constants.Intake.RollerkI;
        rollerMotorSlot0Config.kD = Constants.Intake.RollerkD;
        rollerMotorSlot0Config.kS = Constants.Intake.RollerkS;
        rollerMotorSlot0Config.kG = Constants.Intake.RollerkG;

        rollerMotor.getConfigurator().apply(rollerMotorConfiguration);
        rollerMotor.getConfigurator().apply(rollerMotorMotionMagicConfig);
        rollerMotor.getConfigurator().apply(rollerMotorSlot0Config);

        pivotMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotMotorConfiguration.CurrentLimits.StatorCurrentLimit = Constants.Intake.PivotCurrentLimit;
        pivotMotorConfiguration.Feedback.RotorToSensorRatio = Constants.Intake.PivotGearRatio;

        pivotMotorMotionMagicConfig.MotionMagicCruiseVelocity = Constants.Intake.PivotMotionMagicCruise;
        pivotMotorMotionMagicConfig.MotionMagicAcceleration = Constants.Intake.PivotMotionMagicAccel;

        pivotMotorSlot0Config.GravityType = GravityTypeValue.Arm_Cosine;
        pivotMotorSlot0Config.kP = Constants.Intake.PivotkP;
        pivotMotorSlot0Config.kI = Constants.Intake.PivotkI;
        pivotMotorSlot0Config.kD = Constants.Intake.PivotkD;
        pivotMotorSlot0Config.kS = Constants.Intake.PivotkS;
        pivotMotorSlot0Config.kG = Constants.Intake.PivotkG;

        pivotMotor.getConfigurator().apply(pivotMotorConfiguration);
        pivotMotor.getConfigurator().apply(pivotMotorMotionMagicConfig);
        pivotMotor.getConfigurator().apply(pivotMotorSlot0Config);

        rollerMotorVelocity = rollerMotor.getVelocity();
        rollerMotorAmperage = rollerMotor.getStatorCurrent();
        rollerMotorTorque = rollerMotor.getTorqueCurrent();

        pivotMotorPosition = pivotMotor.getPosition();
        pivotMotorVelocity = pivotMotor.getVelocity();
        pivotMotorAmperage = pivotMotor.getStatorCurrent();
        pivotMotorTorque = pivotMotor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.Intake.FrequencyUpdateRate,
                rollerMotorVelocity,
                rollerMotorAmperage,
                rollerMotorTorque,
                pivotMotorPosition,
                pivotMotorVelocity,
                pivotMotorAmperage,
                pivotMotorTorque);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        switch (rollerControlMode) {
            case POWER -> {
                rollerMotor.set(rollerTargetPower);
            }
            case POSITION -> {
            }
            case VELOCITY -> {
                rollerMotor.setControl(new MotionMagicVelocityVoltage(rollerTargetVelocity));
            }
        }

        switch (pivotControlMode) {
            case POWER -> {
                pivotMotor.set(pivotTargetPower);
            }
            case POSITION -> {
                pivotMotor.setControl(new MotionMagicVoltage(pivotTargetPosition));
            }
            case VELOCITY -> {
            }
        }

        inputs.currentPivotVelocity = pivotMotorVelocity.getValueAsDouble();
        inputs.currentPivotPosition = pivotMotorPosition.getValueAsDouble();
        inputs.currentPivotAmperage = pivotMotorAmperage.getValueAsDouble();
        inputs.currentPivotTorque = pivotMotorTorque.getValueAsDouble();
        inputs.targetPivotPosition = this.pivotTargetPosition;

        inputs.currentRollerVelocity = this.rollerTargetVelocity;
        inputs.currentRollerAmperage = rollerMotorAmperage.getValueAsDouble();
        inputs.currentRollerTorque = rollerMotorTorque.getValueAsDouble();
    }

    @Override
    public void setPivotSpeed(double speed) {
        this.pivotControlMode = ControlMode.POWER;
        pivotTargetPower = speed;
    }

    @Override
    public void setPivotPosition(double position) {
        this.pivotControlMode = ControlMode.POSITION;
        pivotTargetPosition = position;
    }

    @Override
    public void setRollerSpeed(double velocity) {
        this.rollerControlMode = ControlMode.VELOCITY;
        rollerTargetPower = velocity;
    }

    @Override
    public void setRollerVelocity(double velocity) {
        this.rollerControlMode = ControlMode.VELOCITY;
        rollerTargetVelocity = velocity;
    }
}
