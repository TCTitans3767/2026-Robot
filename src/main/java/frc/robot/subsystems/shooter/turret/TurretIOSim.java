package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class TurretIOSim implements TurretIO{
    private enum ControlMode {
        POWER,
        POSITION
    }

    private ControlMode controlMode = ControlMode.POWER;

    private final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);
    private final DCMotorSim sim =
            new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, 1), gearbox);

    private double rotationTarget = 0;
    private double powerTarget = 0;

    private PIDController pid = new PIDController(Constants.Shooter.Turret.simP, Constants.Shooter.Turret.simI, Constants.Shooter.Turret.simD);

    public TurretIOSim() {}

    @Override
    public void updateInputs(TurretIOInputs inputs) {

        switch (this.controlMode) {
            case POWER -> sim.setInputVoltage(12 * this.powerTarget);
            case POSITION -> sim.setInputVoltage(MathUtil.clamp(pid.calculate(sim.getAngularPositionRad()), -12.0, 12.0));
        }

        if (inputs.atRightLimit || inputs.atLeftLimit) {
            sim.setAngularVelocity(0);
            sim.setInputVoltage(0);
        }

        sim.update(Constants.loopPeriodSecs);

        inputs.targetRotation = this.rotationTarget;
        inputs.targetPower = this.powerTarget;
        inputs.currentAmperage = sim.getInputVoltage();
        inputs.currentRotation = sim.getAngularPositionRad();
        inputs.currentTorque = sim.getTorqueNewtonMeters();
        inputs.atLeftLimit = sim.getAngularPositionRad() - Robot.drivetrain.getRotation().getRadians() >= (Math.PI - 0.1);
        inputs.atRightLimit =  sim.getAngularPositionRad() - Robot.drivetrain.getRotation().getRadians() <= -(Math.PI + 0.1);
    };

    @Override
    public void setRotation(double targetRotation) {
        this.controlMode = ControlMode.POSITION;
        rotationTarget = targetRotation;
        if (targetRotation >= Constants.Shooter.Turret.leftLimit) {
            pid.setSetpoint(targetRotation - 0.01);
        } else if (targetRotation <= Constants.Shooter.Turret.rightLimit) {
            pid.setSetpoint(targetRotation + 0.01);
        } else {
            pid.setSetpoint(targetRotation);
        }
    };

    @Override
    public void setPower(double percentage) {
        this.controlMode = ControlMode.POWER;
        this.powerTarget = percentage;
    };

}
