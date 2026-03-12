package frc.robot.commands.states;

import ControlAnnotations.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.subsystems.shooter.ShooterStack;
import frc.robot.util.HubState;
import frc.robot.utils.RobotTransitions;

import static edu.wpi.first.units.Units.Seconds;

@State
public class NeutralZoneState extends Command {

    public NeutralZoneState() {

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
//        if (TriggerBoard.isRobotInNoShootingZone()) {
//            inactive();
//        } else {
//            if (TriggerBoard.isHubActive()) {
//                active();
//            } else {
//                inactive();
//            }
//        }
//
//        if (TriggerBoard.isShiftSoon() && !HubState.isActiveNext()) {
//            RobotControl.setCurrentMode(RobotTransitions.hubInactiveTransition);
//            return;
//        } else if (TriggerBoard.isShiftSoon() && HubState.isActiveNext()) {
//            RobotControl.setCurrentMode(RobotTransitions.hubActiveTransition);
//            return;
//        }

        if (TriggerBoard.isOnSide()) {
            RobotControl.setCurrentMode(RobotTransitions.onSideEnterTransition);
            return;
        }

        if (TriggerBoard.leftOfHub()) {
            if (Robot.getAlliance() == DriverStation.Alliance.Blue) {
                Robot.shooterArray.setTarget(Constants.FieldPoses.blueLeftPassTarget);
            } else {
                Robot.shooterArray.setTarget(Constants.FieldPoses.redLeftPassTarget);
            }
        } else {
            if (Robot.getAlliance() == DriverStation.Alliance.Blue) {
                Robot.shooterArray.setTarget(Constants.FieldPoses.blueRightPassTarget);
            } else {
                Robot.shooterArray.setTarget(Constants.FieldPoses.redRightPassTarget);
            }
        }

        // Shooting mode
        if (TriggerBoard.isShootButtonPressed()) {
            Robot.shooterArray.enableShooting(true);
            Robot.intake.setRollerVelocity(20);
            Robot.indexer.setIndexVelocity(30);
        } else {
            Robot.shooterArray.enableShooting(false);
            Robot.indexer.setIndexVelocity(0);
        }

        // Intaking mode
        if (TriggerBoard.isIntakeButtonPressed()) {
            Robot.intake.setPivotPosition(0);
            Robot.intake.setRollerVelocity(40);
        } else {
            Robot.intake.setPivotPosition(0.15);
        }

        // If neither intaking mode nor shooting mode is active, set intake roller speed to 0
        if (!TriggerBoard.isShootButtonPressed() && !TriggerBoard.isIntakeButtonPressed()) {
            Robot.intake.setRollerSpeed(0);
        }

//        if (TriggerBoard.isOffSide()) {
//            RobotControl.setCurrentMode(RobotTransitions.offSideEnterTransition);
//            return;
//        }
    }

    private void active() {
        if (TriggerBoard.leftOfHub()) {
            if (Robot.getAlliance() == DriverStation.Alliance.Blue) {
                Robot.shooterArray.setTarget(Constants.FieldPoses.blueLeftPassTarget);
            } else {
                Robot.shooterArray.setTarget(Constants.FieldPoses.redLeftPassTarget);
            }
        } else {
            if (Robot.getAlliance() == DriverStation.Alliance.Red) {
                Robot.shooterArray.setTarget(Constants.FieldPoses.blueRightPassTarget);
            } else {
                Robot.shooterArray.setTarget(Constants.FieldPoses.redRightPassTarget);
            }
        }
    }

    private void inactive() {
        active();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
