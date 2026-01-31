package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FlywheelIOSim implements FlywheelIO{

    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
    private final DCMotorSim sim =
            new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.015, 1.1), gearbox);

    private double velocityTarget = 0;

    private PIDController pid = new PIDController(Constants.Shooter.Flywheel.simP, Constants.Shooter.Flywheel.simI, Constants.Shooter.Flywheel.simD);

    public FlywheelIOSim() {}

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        sim.setInputVoltage(MathUtil.clamp(pid.calculate(Units.radiansToRotations(sim.getAngularVelocityRadPerSec())) + (pid.getError() < 0 ? -Constants.Shooter.Flywheel.simS : Constants.Shooter.Flywheel.simS), -12, 12));
        sim.update(Constants.loopPeriodSecs);

        inputs.currentAmperage = sim.getCurrentDrawAmps();
        inputs.currentTorque = sim.getTorqueNewtonMeters();
        inputs.currentVelocityRPS = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
        inputs.targetVelocity = this.velocityTarget;
    }

    @Override
    public void setVelocity(double rotationsPerSecond) {
        this.velocityTarget = rotationsPerSecond;
        pid.setSetpoint(rotationsPerSecond);
    }
}
