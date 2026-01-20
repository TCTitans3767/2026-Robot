package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

        if (flywheelTouchingFuel() && (!wasTouchingFuel)) {
            wasTouchingFuel = true;
        } else if (!flywheelTouchingFuel() && (wasTouchingFuel)) {
            wasTouchingFuel = false;
            fuelShotNumber++;
        }
    }

    public void setVelocity(double rotationsPerSecond) {
        io.setVelocity(rotationsPerSecond);
    }

    public void setPower(double percent) {
        io.setPower(percent);
    }

    public double getVelocity() {
        return inputs.currentVelocity;
    }

    public double getTargetVelocity() {
        return inputs.targetVelocity;
    }

    private boolean flywheelTouchingFuel() {
        return inputs.currentAmperage > Constants.Shooter.amperageThreshold;
    }

    public void resetFuelShotNumber() {
        fuelShotNumber = 0;
    }
}
