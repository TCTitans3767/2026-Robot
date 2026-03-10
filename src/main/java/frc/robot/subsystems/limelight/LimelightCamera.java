package frc.robot.subsystems.limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import limelight.Limelight;
import limelight.networktables.*;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

public class LimelightCamera extends SubsystemBase implements LimelightCameraIO {

    public enum limelightPipeline {
        APRIL_TAG,
        OBJECT_DETECTION,
        OPEN_CV,
        COLOR
    }

    private final LimelightCameraIOInputsAutoLogged inputs = new LimelightCameraIOInputsAutoLogged();
    private final LimelightCameraIO io;

    private int megaTag1Estimations = 0;
    private LimelightPoseEstimator megaTagOneEstimator;
    private LimelightPoseEstimator megaTagTwoEstimator;
    private LimelightSettings.ImuMode currentIMUMode;
    private Pose3d mostRecentPoseEstimate = new Pose3d();
    private boolean isMostRecentPoseEstimateValid = false;
    private double mostRecentAmbiguity = 0;
    private ArrayList<Integer> currentVisibleTags;

    private final String limelightName;
    private final Pose3d robotToLimelight;
    private limelightPipeline currentPipeline;

    private final Limelight camera;

    public LimelightCamera(String limelightName, Pose3d robotToLimelight, limelightPipeline initialPipeline) {
        this.limelightName = limelightName;
        this.robotToLimelight = robotToLimelight;
        this.currentPipeline = initialPipeline;

        camera = new Limelight(limelightName);

        camera.getSettings().withLimelightLEDMode(LimelightSettings.LEDMode.PipelineControl)
                .withCameraOffset(robotToLimelight)
                .save();

        megaTagOneEstimator = camera.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG1);
        megaTagTwoEstimator = camera.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG2);

        this.io = this;
    }

    @Override
    public void updateInputs(LimelightCameraIOInputs inputs) {
        inputs.isMostRecentEstimateValid = this.isMostRecentPoseEstimateValid;
        inputs.mostRecentPoseEstimate = this.mostRecentPoseEstimate;
        inputs.mostRecentAmbiguity = this.mostRecentAmbiguity;
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs(this.limelightName, inputs);
        switch (this.currentPipeline) {
            case APRIL_TAG -> {
                aprilTagPeriodic();
            }
            case OBJECT_DETECTION -> {
                objectDetectionPeriodic();
            }
            case OPEN_CV -> {
                openCVPeriodic();
            }
            case COLOR -> {
                colorPeriodic();
            }
        }
    }

    private void colorPeriodic() {
    }

    private void openCVPeriodic() {

    }

    private void aprilTagPeriodic() {
        if (megaTag1Estimations < Constants.Limelights.minMegaTagOneEstimations) {
            if (this.currentIMUMode != LimelightSettings.ImuMode.SyncInternalImu) {
                camera.getSettings().withImuMode(LimelightSettings.ImuMode.SyncInternalImu);
                this.currentIMUMode =  LimelightSettings.ImuMode.SyncInternalImu;
            }

            Orientation3d robotOrientation = new Orientation3d(
                    new Rotation3d(Robot.drivetrain.getRotation()),
                    new AngularVelocity3d(Units.DegreesPerSecond.of(0), Units.DegreesPerSecond.of(0), Units.DegreesPerSecond.of(0))
            );

            camera.getSettings().withRobotOrientation(robotOrientation).save();

            Optional<PoseEstimate> poseEstimate = megaTagOneEstimator.getPoseEstimate();

            poseEstimate.ifPresent((estimate) -> {
                this.mostRecentPoseEstimate = estimate.pose;
                this.mostRecentAmbiguity = estimate.getAvgTagAmbiguity();
                if (!(estimate.getAvgTagAmbiguity() > Constants.Limelights.maxAmbiguity)) {
                    Robot.drivetrain.addVisionMeasurement(estimate.pose.toPose2d(), Timer.getFPGATimestamp(), VecBuilder.fill(0, 0, 0));
                    megaTag1Estimations++;
                    this.isMostRecentPoseEstimateValid = true;
                } else {
                    this.isMostRecentPoseEstimateValid = false;
                }
            });
        } else {
            if (this.currentIMUMode != LimelightSettings.ImuMode.InternalImu) {
                camera.getSettings().withImuMode(LimelightSettings.ImuMode.InternalImu);
                this.currentIMUMode = LimelightSettings.ImuMode.InternalImu;
            }

//            Orientation3d robotOrientation = new Orientation3d(
//                    new Rotation3d(0, 0, Robot.drivetrain.getRotation().getRadians()),
//                    Robot.drivetrain.getPigeon2().getAngularVelocityZWorld().getValue(),
//                    Robot.drivetrain.getPigeon2().getAngularVelocityYWorld().getValue(),
//                    Robot.drivetrain.getPigeon2().getAngularVelocityXWorld().getValue()
//            );
//
//            camera.getSettings().withRobotOrientation(robotOrientation).save();

            Optional<PoseEstimate> poseEstimate = LimelightPoseEstimator.BotPose.BLUE_MEGATAG2.get(this.camera);

            poseEstimate.ifPresent((estimate) -> {
                this.mostRecentPoseEstimate = estimate.pose;
                if (!(estimate.getMaxTagAmbiguity() > Constants.Limelights.maxAmbiguity)) {
                    Robot.drivetrain.addVisionMeasurement(estimate.pose.toPose2d());
                    this.isMostRecentPoseEstimateValid = true;
                } else {
                    this.isMostRecentPoseEstimateValid = false;
                }
            });
        }
    }

    private void objectDetectionPeriodic() {

    }
}
