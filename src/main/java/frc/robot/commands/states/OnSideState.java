package frc.robot.commands.states;

import ControlAnnotations.State;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.commands.basicCommands.intake.IntakeRollerDriveVelocityMatch;
import frc.robot.commands.transitions.OnSideEnterTransition;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.util.HubState;
import frc.robot.utils.RobotTransitions;

import static edu.wpi.first.units.Units.Seconds;

@State
public class OnSideState extends Command {

    public OnSideState() {

    }

    @Override
    public void initialize() {
//        Robot.indexer.setIndexVelocity(15);
//        Robot.intake.setPivotPosition(0);
//        Robot.intake.setRollerVelocitySupplier(() -> MathUtil.clamp(Math.sqrt(((Robot.drivetrain.getChassisSpeeds().vxMetersPerSecond * Robot.drivetrain.getChassisSpeeds().vxMetersPerSecond) + (Robot.drivetrain.getChassisSpeeds().vyMetersPerSecond * Robot.drivetrain.getChassisSpeeds().vyMetersPerSecond)) / Constants.Intake.rollerRadius) / (2 * Math.PI), 40.0, 100));
        Robot.shooterArray.setTarget(Robot.getAlliance() == DriverStation.Alliance.Blue ? Constants.FieldPoses.blueHub : Constants.FieldPoses.redHub);
        Robot.shooterArray.enableTragetPointing();
        Robot.shooterArray.setInterpolationMaps(Constants.Shooter.hoodAngleInterpolationMap, Constants.Shooter.flywheelVelocityInterpolationMap);
    }

    @Override
    public void execute() {
//        if (TriggerBoard.isRobotInNoShootingZone()) {
//            inactive();
//        } else {
//            if (HubState.isActive()) {
//                active();
//            } else {
//                inactive();
//            }
//        }

//        if (HubState.timeRemainingInCurrentShift().orElse(Seconds.of(Constants.shiftOffset + 1)).in(Seconds) <= Constants.shiftOffset && !HubState.isActiveNext()) {
//            RobotControl.setCurrentMode(RobotTransitions.hubInactiveTransition);
//            return;
//        } else if (HubState.timeRemainingInCurrentShift().orElse(Seconds.of(Constants.shiftOffset + 1)).in(Seconds) <= Constants.shiftOffset && HubState.isActiveNext()) {
//            RobotControl.setCurrentMode(RobotTransitions.hubActiveTransition);
//            return;
//        }

        if (TriggerBoard.isInNeutralZone()) {
            RobotControl.setCurrentMode(RobotTransitions.neutralZoneEnterTransition);
            return;
        }
//        if (Robot.driverController.getLeftTriggerAxis() > 0.5) {
//            // Intake mode
//            Robot.shooterArray.enableShooting(false);
//            Robot.intake.setRollerVelocity(50);
//            Robot.indexer.setIndexVelocity(15);
//            Robot.intake.setPivotPosition(0);
//        } else if (Robot.driverController.getRightTriggerAxis() > 0.5) {
//            // Shoot mode
//            Robot.shooterArray.enableShooting(true);
//            Robot.intake.setRollerVelocity(50);
//            Robot.indexer.setIndexVelocity(15);
//            Robot.intake.setPivotPosition(0.15);
//        } else {
//            // Neutral mode
//            Robot.shooterArray.enableShooting(false);
//            Robot.intake.setRollerVelocity(0);
//            Robot.indexer.setIndexVelocity(0);
//        }
//        System.out.println("Ran Active function from OnSideState");
//        active();
//        System.out.println("Ran execute from OnSideState");
        if (TriggerBoard.isSpitButtonPressed()) {
            Robot.intake.setRollerVelocity(-50);
            Robot.indexer.setIndexVelocity(-20);
            return;
        }

        if (TriggerBoard.isShootButtonPressed()) {
            Robot.shooterArray.enableShooting(true);
            Robot.intake.setRollerVelocity(20);
            Robot.indexer.setIndexVelocity(30);
            Robot.intake.setPivotPosition(MathUtil.isNear(0, Timer.getFPGATimestamp() % Constants.Intake.bumpFrequency, Constants.Intake.bumpTimerDeadband) ? Constants.Intake.bottomPosition : Constants.Intake.bumpPosition);
        } else {
            Robot.shooterArray.enableShooting(false);
        }

        if (TriggerBoard.isIntakeButtonPressed()) {
            Robot.intake.setPivotPosition(-0.2);
            Robot.intake.setRollerVelocity(50);
            Robot.indexer.setIndexVelocity(20);
        } else {
//            Robot.intake.setPivotPosition(0.15);
        }

        if (!TriggerBoard.isShootButtonPressed() && !TriggerBoard.isIntakeButtonPressed()) {
            Robot.intake.setRollerSpeed(0);
            Robot.indexer.setIndexSpeed(0);
        }
    }

    public void active() {
    }

    public void inactive() {
        Robot.shooterArray.enableShooting(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}