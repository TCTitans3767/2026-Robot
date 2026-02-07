package frc.robot.commands.transitions;

import ControlAnnotations.Transition;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetMode;
import frc.robot.commands.basicCommands.shooterStack.DisableShooting;
import frc.robot.utils.RobotStates;

@Transition
public class OffSideEnterTransition extends SequentialCommandGroup {

    public OffSideEnterTransition() {
        addCommands(
                new DisableShooting(),
                new SetMode(RobotStates.offSideState)
        );
    }

}
