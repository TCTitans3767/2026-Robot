package frc.robot.subsystems.shooter.hood;

import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import frc.robot.Constants;
import frc.robot.Robot;

public class HoodIOCompetition implements HoodIO{

    private final int servoChannel;
    private final ServoChannelConfig channelConfig;
    private final ServoChannel servo;

    private double angleTarget = 0;
    private int targetPulseWidth = 1000;

    public HoodIOCompetition(int servoChannel) {
        this.servoChannel = servoChannel;
        this.channelConfig = new ServoChannelConfig(ServoChannel.ChannelId.fromInt(servoChannel));
        this.servo = Robot.servoHub.getServoChannel(ServoChannel.ChannelId.fromInt(servoChannel));

        channelConfig.disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kDoNotSupplyPower);
        channelConfig.pulseRange(Constants.Shooter.Hood.minimumPulseWidth, 1500, Constants.Shooter.Hood.maximumPulseWidth);

        ServoHubConfig servoHubConfig = new ServoHubConfig();

        servoHubConfig.apply(ServoChannel.ChannelId.fromInt(servoChannel), channelConfig);

        Robot.servoHub.configure(servoHubConfig, ResetMode.kNoResetSafeParameters);

        this.servo.setPowered(true);

        this.servo.setEnabled(true);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        this.servo.setPulseWidth(this.targetPulseWidth);

        inputs.targetAngle = this.angleTarget;
        inputs.commandedPulseWidth = this.targetPulseWidth;
        inputs.currentAngle = this.servo.getPulseWidth();
    }

    @Override
    public void setAngle(double angle) {
//        this.angleTarget = angle;
//        this.servoPulseWidth = angleToPulseWidth(angle);
        this.targetPulseWidth = (int) angle;
    }

    // TODO: give this function an actual output that does what it should
    private int angleToPulseWidth(double angle) {
        return 1000;
    }

    private double pulseWidthToAngle(int pulseWidth) {return 0.0;}
}
