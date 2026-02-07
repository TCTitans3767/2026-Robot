package frc.robot.commands.basicCommands.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeRollerDriveVelocityMatch extends Command {

    public IntakeRollerDriveVelocityMatch() {

    }

    @Override
    public void initialize() {
        Robot.intake.setRollerVelocitySupplier(() -> MathUtil.clamp(Math.sqrt(((Robot.drivetrain.getChassisSpeeds().vxMetersPerSecond * Robot.drivetrain.getChassisSpeeds().vxMetersPerSecond) + (Robot.drivetrain.getChassisSpeeds().vyMetersPerSecond * Robot.drivetrain.getChassisSpeeds().vyMetersPerSecond)) / Constants.Intake.rollerRadius) / (2 * Math.PI), 5.0, 100));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
