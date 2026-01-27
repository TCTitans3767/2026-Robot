package frc.robot.subsystems.shooter.hood;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.config.ServoChannelConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Robot;

public class HoodIOCompetition implements HoodIO{

    private final int leftServoChannel, rightServoChannel;
    private final ServoChannelConfig leftServoChannelConfig, rightServoChannelConfig;
    private final ServoChannel leftServo, rightServo;

    private double angleTarget = 0;
    private int servoPulseWidth = 1000;

    public HoodIOCompetition(int leftServoChannel, int rightServoChannel) {
        this.leftServoChannel = leftServoChannel;
        this.leftServoChannelConfig = new ServoChannelConfig(ServoChannel.ChannelId.fromInt(leftServoChannel));
        this.leftServo = Robot.servoHub.getServoChannel(ServoChannel.ChannelId.fromInt(leftServoChannel));
        this.rightServoChannel = rightServoChannel;
        this.rightServoChannelConfig = new ServoChannelConfig(ServoChannel.ChannelId.fromInt(rightServoChannel));
        this.rightServo = Robot.servoHub.getServoChannel(ServoChannel.ChannelId.fromInt(rightServoChannel));

        leftServoChannelConfig.disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kDoNotSupplyPower);
        leftServoChannelConfig.pulseRange(Constants.Shooter.Hood.minimumPulseWidth, 1500, Constants.Shooter.Hood.maximumPulseWidth);
        rightServoChannelConfig.disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kDoNotSupplyPower);
        rightServoChannelConfig.pulseRange(Constants.Shooter.Hood.minimumPulseWidth, 1500, Constants.Shooter.Hood.maximumPulseWidth);

        this.leftServo.setPowered(true);
        this.rightServo.setPowered(true);

        this.leftServo.setEnabled(true);
        this.rightServo.setEnabled(true);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        leftServo.setPulseWidth(MathUtil.clamp(servoPulseWidth, Constants.Shooter.Hood.minimumPulseWidth, Constants.Shooter.Hood.maximumPulseWidth));
        rightServo.setPulseWidth(MathUtil.clamp(servoPulseWidth, Constants.Shooter.Hood.minimumPulseWidth, Constants.Shooter.Hood.maximumPulseWidth));

        inputs.targetAngle = this.angleTarget;
        inputs.commandedPulseWidth = this.servoPulseWidth;
        inputs.currentAngle = this.leftServo.getPulseWidth();
    }

    @Override
    public void setAngle(double angle) {
        this.angleTarget = angle;
        this.servoPulseWidth = angleToPulseWidth(angle);
    }

    // TODO: give this function an actual output that does what it should
    private int angleToPulseWidth(double angle) {
        return 1000;
    }
}
