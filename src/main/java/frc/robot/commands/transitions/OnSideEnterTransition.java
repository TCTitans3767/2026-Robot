package frc.robot.commands.transitions;

import ControlAnnotations.Transition;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetMode;
import frc.robot.commands.basicCommands.shooterStack.DisableShooting;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotStates;

@Transition
public class OnSideEnterTransition extends SequentialCommandGroup {

    public OnSideEnterTransition() {
        addCommands(
                new DisableShooting(),
                new SetMode(RobotStates.onSideState)
        );
    }
}
