package frc.robot.commands.transitions;

import ControlAnnotations.Transition;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.SetMode;
import frc.robot.subsystems.robotControl.RobotControl;

@Transition
public class HubInactiveTransition extends SequentialCommandGroup {

    public HubInactiveTransition() {

        addCommands(
                new SetMode(RobotControl.getPreviousMode())
        );

    }

}
