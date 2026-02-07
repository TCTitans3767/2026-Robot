package frc.robot.commands.states;

import ControlAnnotations.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.commands.transitions.OnSideEnterTransition;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.util.HubState;
import frc.robot.utils.RobotTransitions;

@State
public class OnSideState extends Command {

    public OnSideState() {

    }

    @Override
    public void initialize() {
        // TODO: do this
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

        if (TriggerBoard.isInNeutralZone()) {
            RobotControl.setCurrentMode(RobotTransitions.neutralZoneEnterTransition);
            return;
        }
    }

    public void active() {
        Robot.shooterArray.enableShooting(true);
    }

    public void inactive() {
        Robot.shooterArray.enableShooting(false);
    }


}
