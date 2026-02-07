package frc.robot.commands.states;

import ControlAnnotations.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.util.HubState;
import frc.robot.utils.RobotTransitions;

@State
public class NeutralZoneState extends Command {

    public NeutralZoneState() {

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (HubState.isActive()) {
            active();
        } else {
            inactive();
        }

        if (HubState.timeRemainingInCurrentShift() <= Constants.shiftOffset && !HubState.isActiveNext()) {
            RobotControl.setCurrentMode(RobotTransitions.hubInactiveTransition);
            return;
        } else if (HubState.timeRemainingInCurrentShift() <= Constants.shiftOffset && HubState.isActiveNext()) {
            RobotControl.setCurrentMode(RobotTransitions.hubActiveTransition);
            return;
        }

        if (TriggerBoard.isOnSide()) {
            RobotControl.setCurrentMode(RobotTransitions.onSideEnterTransition);
            return;
        }

        if (TriggerBoard.isOffSide()) {
            RobotControl.setCurrentMode(RobotTransitions.offSideTransition);
            return;
        }
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
}
