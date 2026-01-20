package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class HoodIOSim implements HoodIO{
    private final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);
    private final DCMotorSim sim =
            new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, 1), gearbox);

    private double angleTarget = 0;

    private PIDController pid = new PIDController(Constants.Shooter.Hood.simP, Constants.Shooter.Hood.simI, Constants.Shooter.Hood.simD);

    public HoodIOSim() {};

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        sim.setInputVoltage(MathUtil.clamp(pid.calculate(sim.getAngularPositionRad()), -12, 12));
        sim.update(Constants.loopPeriodSecs);

        inputs.currentAngle = -sim.getAngularPositionRad();
        inputs.currentAmperage = sim.getCurrentDrawAmps();
        inputs.currentTorque = sim.getTorqueNewtonMeters();
        inputs.targetAngle = -this.angleTarget;
    }

    @Override
    public void setAngle(double angle) {
        this.angleTarget = -angle;
        pid.setSetpoint(-angle);
    }

    @Override
    public void setPower(double percentage) {
        return;
    }
}
