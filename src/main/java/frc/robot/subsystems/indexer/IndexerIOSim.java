package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;

public class IndexerIOSim implements IndexerIO{

    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
    private final DCMotorSim sim =
            new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.015, 1.1), gearbox);

    private double velocityTarget = 0;

    private PIDController pid = new PIDController(Constants.Indexer.simP, Constants.Indexer.simI, Constants.Indexer.simD);

    public IndexerIOSim() {}

    @Override
    public void updateInputs(IndexerIO.IndexerIOInputs inputs) {
        sim.setInputVoltage(MathUtil.clamp(pid.calculate(Units.radiansToRotations(sim.getAngularVelocityRadPerSec())) + (pid.getError() < 0 ? -Constants.Indexer.simS : Constants.Indexer.simS), -12, 12));
        sim.update(Constants.loopPeriodSecs);

        inputs.currentAmperage = sim.getCurrentDrawAmps();
        inputs.currentTorque = sim.getTorqueNewtonMeters();
        inputs.currentVelocity = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
        inputs.targetVelocity = this.velocityTarget;
    }

    @Override
    public void setVelocity(double rotationsPerSecond) {
        this.velocityTarget = rotationsPerSecond;
        pid.setSetpoint(rotationsPerSecond);
    }
}
