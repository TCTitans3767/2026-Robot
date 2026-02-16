package frc.robot.commands.transitions;

import ControlAnnotations.Transition;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetMode;
import frc.robot.commands.basicCommands.shooterStack.EnableShooting;
import frc.robot.commands.basicCommands.shooterStack.SetShooterTarget;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.util.HubState;

@Transition
public class HubActiveTransition extends SequentialCommandGroup {

    public HubActiveTransition() {
        addCommands(
                new SetMode(RobotControl.getPreviousMode())
        );
    }

}
