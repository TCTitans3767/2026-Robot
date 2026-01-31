package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.feeder.FeederIO;
import frc.robot.subsystems.intake.IntakeIO;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final IntakeIO io;
    private final String name;

    public Intake(String name, IntakeIO io) {
        this.io = io;
        this.name = name;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
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
        io.setRollerVelocity(velocity);
    }

}
