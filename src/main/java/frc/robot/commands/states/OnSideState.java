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

import static edu.wpi.first.units.Units.Seconds;

@State
public class OnSideState extends Command {

    public OnSideState() {

    }

    @Override
    public void initialize() {
        Robot.indexer.setIndexVelocity(15);
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
//
//        if (HubState.timeRemainingInCurrentShift().get().in(Seconds) <= Constants.shiftOffset && !HubState.isActiveNext()) {
//            RobotControl.setCurrentMode(RobotTransitions.hubInactiveTransition);
//            return;
//        } else if (HubState.timeRemainingInCurrentShift().get().in(Seconds) <= Constants.shiftOffset && HubState.isActiveNext()) {
//            RobotControl.setCurrentMode(RobotTransitions.hubActiveTransition);
//            return;
//        }

//        if (TriggerBoard.isInNeutralZone()) {
//            RobotControl.setCurrentMode(RobotTransitions.neutralZoneEnterTransition);
//            return;
//        }
        active();
    }

    public void active() {
        Robot.shooterArray.enableShooting(Robot.driverController.getRightTriggerAxis() > 0.5);
    }

    public void inactive() {
        Robot.shooterArray.enableShooting(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
