package frc.robot.subsystems.shooter.turret;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;


public class TurretIOCompetition implements TurretIO{

    private enum ControlMode {
        POWER,
        POSITION
    }

    private final TalonFX turretMotor;
    private final TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
    private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    private final Slot0Configs slot0Configs = new Slot0Configs();
    private final int CANID;

    private double targetRotation = 0;
    private double targetPower = 0;
    private ControlMode controlMode = ControlMode.POWER;

    private final StatusSignal<Angle> rotationStatusSignal;
    private final StatusSignal<Current> amperageStatusSignal;
    private final StatusSignal<Current> torqueStatusSignal;
    private final StatusSignal<AngularVelocity> velocityStatusSignal;

    public TurretIOCompetition(int CANID) {
        this.CANID = CANID;
        turretMotor = new TalonFX(CANID);

        turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turretMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.Turret.currentLimit;
        turretMotorConfig.Feedback.RotorToSensorRatio = Constants.Shooter.Turret.gearRatio;

        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Shooter.Turret.motionMagicCruise;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.Turret.motionMagicAccel;

        slot0Configs.kP = Constants.Shooter.Turret.compP;
        slot0Configs.kI = Constants.Shooter.Turret.compI;
        slot0Configs.kD = Constants.Shooter.Turret.compD;
        slot0Configs.kS = Constants.Shooter.Turret.compS;

        turretMotor.getConfigurator().apply(turretMotorConfig);
        turretMotor.getConfigurator().apply(motionMagicConfigs);
        turretMotor.getConfigurator().apply(slot0Configs);

        rotationStatusSignal = turretMotor.getPosition();
        amperageStatusSignal = turretMotor.getStatorCurrent();
        torqueStatusSignal = turretMotor.getTorqueCurrent();
        velocityStatusSignal = turretMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(Constants.Shooter.signalUpdateFrequency, rotationStatusSignal, amperageStatusSignal, torqueStatusSignal, velocityStatusSignal);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {

        switch (this.controlMode) {
            case POWER -> turretMotor.set(targetPower);
            case POSITION -> turretMotor.setControl(new MotionMagicVoltage(this.targetRotation));
        }

        inputs.targetRotation = this.targetRotation;
        inputs.currentRotation = rotationStatusSignal.getValueAsDouble();
        inputs.currentAmperage = amperageStatusSignal.getValueAsDouble();
        inputs.currentTorque = torqueStatusSignal.getValueAsDouble();
        inputs.currentVelocity = velocityStatusSignal.getValueAsDouble();
    }

    @Override
    public void setRotation(double targetRotation) {
        this.controlMode = ControlMode.POSITION;
        this.targetRotation = Units.radiansToRotations(targetRotation);
    }

    @Override
    public void setPower(double percentage) {
        this.controlMode = ControlMode.POWER;
        this.targetPower = percentage * 255;
    }

    private double lastTurretAngle = 0.0;

    public double calculateTurretAngleFromCANCoderDegrees(double e1, double e2) {
        double difference = e2 - e1;
        Logger.recordOutput(this.CANID + "/e1 turret", e1);
        Logger.recordOutput(this.CANID + "/e2 turret", e2);
        Logger.recordOutput(this.CANID + "/OG Diff", difference);

        // System.out.println("OG Diff: " + differenceV2);
        if (difference > 250) {
            difference -= 360;
        }
        if (difference < -250) {
            difference += 360;
        }
        difference *= Constants.Shooter.Turret.SLOPE;

        double e1Rotations = (difference * Constants.Shooter.Turret.GEAR_0_TOOTH_COUNT / Constants.Shooter.Turret.GEAR_1_TOOTH_COUNT) / 360.0;
        double e1RotationsFloored = Math.floor(e1Rotations);
        double turretAngle = (e1RotationsFloored * 360.0 + e1) * (Constants.Shooter.Turret.GEAR_1_TOOTH_COUNT / Constants.Shooter.Turret.GEAR_0_TOOTH_COUNT);
        Logger.recordOutput(this.CANID + "/e1 Rotations", e1Rotations);
        Logger.recordOutput(this.CANID + "/in method turret", turretAngle);
        Logger.recordOutput(this.CANID + "/BUNZ difference", difference);
        Logger.recordOutput(this.CANID + "/floored e1", e1RotationsFloored);

        double rotation = (Constants.Shooter.Turret.GEAR_1_TOOTH_COUNT / Constants.Shooter.Turret.GEAR_0_TOOTH_COUNT) * 360.0;

        double a0 = turretAngle;
        double aPlus = turretAngle + rotation;
        double aMinus = turretAngle - rotation;

        double error0 = Math.abs(a0 - lastTurretAngle);
        double errorPlus = Math.abs(aPlus - lastTurretAngle);
        double errorMinus = Math.abs(aMinus - lastTurretAngle);

        if (errorPlus < error0 && errorPlus < errorMinus) {
            turretAngle = aPlus;
        } else if (errorMinus < error0){
            turretAngle = aMinus;
        }
        lastTurretAngle = turretAngle;
        return turretAngle;
    }}
