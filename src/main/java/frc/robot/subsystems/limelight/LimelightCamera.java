package frc.robot.subsystems.limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import limelight.Limelight;
import limelight.networktables.*;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

public class LimelightCamera extends SubsystemBase implements LimelightCameraIO {

    public void resetInternalGyro(Pose2d pose) {
        this.camera.getSettings().withImuMode(LimelightSettings.ImuMode.SyncInternalImu).save();

        this.currentIMUMode = LimelightSettings.ImuMode.SyncInternalImu;

        Orientation3d robotOrientation = new Orientation3d(
                new Rotation3d(pose.getRotation()),
                new AngularVelocity3d(Units.DegreesPerSecond.of(0), Units.DegreesPerSecond.of(0), Units.DegreesPerSecond.of(0))
        );

        camera.getSettings().withRobotOrientation(robotOrientation).save();

    }

    public void disableEstimation() {
        this.doEstimation = false;
    }
    public void enableEstimation() {
        this.doEstimation = true;
    }

    public enum limelightPipeline {
        APRIL_TAG,
        OBJECT_DETECTION,
        OPEN_CV,
        COLOR
    }

    private final LimelightCameraIOInputsAutoLogged inputs = new LimelightCameraIOInputsAutoLogged();
    private final LimelightCameraIO io;

    private static int megaTag1Estimations = 0;
    private LimelightPoseEstimator megaTagOneEstimator;
    private LimelightPoseEstimator megaTagTwoEstimator;
    private LimelightSettings.ImuMode currentIMUMode;
    private Pose3d mostRecentPoseEstimate = new Pose3d();
    private boolean isMostRecentPoseEstimateValid = false;
    private double mostRecentAmbiguity = 0;
    private ArrayList<Integer> currentVisibleTags;
    private boolean doEstimation = true;

    private final String limelightName;
    private final Pose3d robotToLimelight;
    private limelightPipeline currentPipeline;

    private final Limelight camera;

    public LimelightCamera(String limelightName, Pose3d robotToLimelight, limelightPipeline initialPipeline) {
        this.limelightName = limelightName;
        this.robotToLimelight = robotToLimelight;
        this.currentPipeline = initialPipeline;

        this.camera = new Limelight(limelightName);

        this.camera.getSettings().withLimelightLEDMode(LimelightSettings.LEDMode.PipelineControl)
                .withCameraOffset(robotToLimelight)
                .save();

        this.megaTagOneEstimator = this.camera.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG1);
        this.megaTagTwoEstimator = this.camera.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG2);

        this.io = this;
    }

    @Override
    public void updateInputs(LimelightCameraIOInputs inputs) {
        inputs.isMostRecentEstimateValid = this.isMostRecentPoseEstimateValid;
        inputs.mostRecentPoseEstimate = this.mostRecentPoseEstimate;
        inputs.mostRecentAmbiguity = this.mostRecentAmbiguity;
        inputs.currentPipeline = this.currentPipeline.toString();
    }

    @Override
    public void periodic() {
        this.updateInputs(this.inputs);
        Logger.processInputs(this.limelightName, this.inputs);
//        System.out.println(this.limelightName + " doEstimation: " + this.doEstimation);
        switch (this.currentPipeline) {
            case APRIL_TAG -> {
                if (this.doEstimation) {
                    aprilTagPeriodic();
                }
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
//        System.out.println(this.limelightName + "APRIL TAG PERIODIC");
        if (megaTag1Estimations < Constants.Limelights.minMegaTagOneEstimations) {
            if (this.currentIMUMode != LimelightSettings.ImuMode.SyncInternalImu) {
                this.camera.getSettings().withImuMode(LimelightSettings.ImuMode.SyncInternalImu);
                this.currentIMUMode =  LimelightSettings.ImuMode.SyncInternalImu;
            }

            Orientation3d robotOrientation = new Orientation3d(
                    new Rotation3d(Robot.drivetrain.getRotation()),
                    new AngularVelocity3d(Units.DegreesPerSecond.of(0), Units.DegreesPerSecond.of(0), Units.DegreesPerSecond.of(0))
            );

            this.camera.getSettings().withRobotOrientation(robotOrientation).save();

            LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.limelightName);

            if (poseEstimate != null) {
//                System.out.println(this.limelightName + " tagCount: " + poseEstimate.tagCount);
                if (poseEstimate.tagCount < 2) {
                    this.isMostRecentPoseEstimateValid = false;
                    return;
                } else {
                    for (LimelightHelpers.RawFiducial tag : poseEstimate.rawFiducials) {
                        if (tag.ambiguity > 0.5) {
                            this.isMostRecentPoseEstimateValid = false;
                            return;
                        }
                    }
                    this.isMostRecentPoseEstimateValid = true;
                    Robot.drivetrain.addVisionMeasurement(poseEstimate.pose, Timer.getFPGATimestamp(), VecBuilder.fill(0, 0, 0));
                    this.mostRecentPoseEstimate = new Pose3d(poseEstimate.pose);
                    megaTag1Estimations++;
                }
            }
//
//            Optional<PoseEstimate> poseEstimate = LimelightPoseEstimator.BotPose.BLUE.get(this.camera);
//
//            poseEstimate.ifPresent((estimate) -> {
////                System.out.println(this.limelightName + " estimate present");
//                System.out.println(this.limelightName + " tagCount: " + estimate.tagCount);
//                if (estimate.tagCount == 0) {
//                    return;
//                }
//                this.mostRecentPoseEstimate = estimate.pose;
//                this.mostRecentAmbiguity = estimate.getMinTagAmbiguity();
//                if (!(estimate.getMinTagAmbiguity() > Constants.Limelights.maxAmbiguity)) {
//                    Robot.drivetrain.addVisionMeasurement(estimate.pose.toPose2d(), Timer.getFPGATimestamp(), VecBuilder.fill(0, 0, 0));
//                    megaTag1Estimations++;
//                    this.isMostRecentPoseEstimateValid = true;
//                } else {
//                    this.isMostRecentPoseEstimateValid = false;
//                }
//            });
        } else {
            if (this.currentIMUMode != LimelightSettings.ImuMode.InternalImu) {
                Orientation3d robotOrientation = new Orientation3d(
                        new Rotation3d(Robot.drivetrain.getRotation()),
                        new AngularVelocity3d(Units.DegreesPerSecond.of(0), Units.DegreesPerSecond.of(0), Units.DegreesPerSecond.of(0))
                );

                this.camera.getSettings().withRobotOrientation(robotOrientation).save();
                this.camera.getSettings().withImuMode(LimelightSettings.ImuMode.InternalImu);
                this.currentIMUMode = LimelightSettings.ImuMode.InternalImu;
            }

            LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.limelightName);

            if (poseEstimate != null) {
//                System.out.println(this.limelightName + " tagCount: " + poseEstimate.tagCount);
                if (poseEstimate.tagCount < 2) {
                    this.isMostRecentPoseEstimateValid = false;
                    return;
                } else {
                    for (LimelightHelpers.RawFiducial tag : poseEstimate.rawFiducials) {
                        if (tag.ambiguity > 0.3) {
                            this.isMostRecentPoseEstimateValid = false;
                            return;
                        }
                    }
                    this.isMostRecentPoseEstimateValid = true;
                    Robot.drivetrain.addVisionMeasurement(poseEstimate.pose, Timer.getFPGATimestamp(), VecBuilder.fill(0, 0, 0));
                    this.mostRecentPoseEstimate = new Pose3d(poseEstimate.pose);
                }
            }

//            Orientation3d robotOrientation = new Orientation3d(
//                    new Rotation3d(0, 0, Robot.drivetrain.getRotation().getRadians()),
//                    Robot.drivetrain.getPigeon2().getAngularVelocityZWorld().getValue(),
//                    Robot.drivetrain.getPigeon2().getAngularVelocityYWorld().getValue(),
//                    Robot.drivetrain.getPigeon2().getAngularVelocityXWorld().getValue()
//            );
//
//            camera.getSettings().withRobotOrientation(robotOrientation).save();

//            Optional<PoseEstimate> poseEstimate = LimelightPoseEstimator.BotPose.BLUE_MEGATAG2.get(this.camera);
//
//            poseEstimate.ifPresent((estimate) -> {
//                this.mostRecentPoseEstimate = estimate.pose;
//                this.mostRecentAmbiguity = estimate.getMinTagAmbiguity();
//                if (estimate.tagCount == 0) {
//                    return;
//                }
//                if (!(estimate.getMinTagAmbiguity() > Constants.Limelights.maxAmbiguity)) {
//                    Robot.drivetrain.addVisionMeasurement(estimate.pose.toPose2d());
//                    this.isMostRecentPoseEstimateValid = true;
//                } else {
//                    this.isMostRecentPoseEstimateValid = false;
//                }
//            });
        }
    }

    private void objectDetectionPeriodic() {

    }

    public static void cancelMegaTagOneEstimation() {
        LimelightCamera.megaTag1Estimations = 50;
    }
}
