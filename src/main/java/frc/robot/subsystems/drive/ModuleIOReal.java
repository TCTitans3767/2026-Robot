package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.Queue;

public class ModuleIOReal extends ModuleIOTalonFX{
    // Queue to read inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    public ModuleIOReal(SwerveModuleConstants constants) {
        super(constants);

        this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        this.drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(super.drivePosition);
        this.turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(super.turnAbsolutePosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
                .mapToDouble(rotations -> Units.rotationsToRadians(rotations) / constants.DriveMotorGearRatio)
                .toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }
}
