package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.feeder.FeederIO;
import frc.robot.subsystems.intake.IntakeIO;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final IntakeIO io;

    private DoubleSupplier velocitySupplier;
    private boolean runRollerWithSupplier = false;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.setRollerVelocity(this.velocitySupplier.getAsDouble());

        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void setPivotSpeed(double speed) {
        io.setPivotSpeed(speed);
    }
    public void setPivotPosition(double position) {
        io.setPivotPosition(position);
    }
    public void setRollerSpeed(double speed) {
        io.setRollerSpeed(speed);
    }
    public void setRollerVelocity(double velocity) {
        runRollerWithSupplier = false;
        io.setRollerVelocity(velocity);
    }
    public void setRollerVelocitySupplier(DoubleSupplier velocitySupplier) {
        runRollerWithSupplier = true;
        this.velocitySupplier = velocitySupplier;
    }

}
