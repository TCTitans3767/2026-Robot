package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface LimelightCameraIO {

    @AutoLog
    public static class LimelightCameraIOInputs {
        public Pose3d mostRecentPoseEstimate = new Pose3d();
        public double mostRecentEstimateTimestamp = 0;
        public boolean isMostRecentEstimateValid = false;
        public int[] visibleTags;
    }

    public default void updateInputs(LimelightCameraIOInputs inputs) {};

}
