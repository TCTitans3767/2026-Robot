package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private final FlywheelIO io;

    private final String name;

    private boolean wasTouchingFuel = false;
    private int fuelShotNumber = 0;

    public Flywheel(String name, FlywheelIO io) {
        this.io = io;
        this.name = name;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    public void setVelocity(double rotationsPerSecond) {
        io.setVelocity(rotationsPerSecond);
    }

    public void setPower(double percent) {
        io.setPower(percent);
    }

    public double getVelocity() {
        return inputs.currentVelocityRPS;
    }

    public double getTargetVelocity() {
        return inputs.targetVelocity;
    }

    public void resetFuelShotNumber() {
        fuelShotNumber = 0;
    }
}
