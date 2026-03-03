package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DrivetrainPublisher {

    private static DoubleSupplier xVelocitySupplier = () -> 0;
    private static DoubleSupplier yVelocitySupplier = () -> 0;
    private static DoubleSupplier thetaVelocitySupplier = () -> 0;
    private static BooleanSupplier inFieldCentricSupplier = () -> true;
    private static BooleanSupplier acceptInputsSupplier = () -> false;

    public DrivetrainPublisher() {
    }

    public static void setInFieldCentricSupplier(BooleanSupplier inFieldCentricSupplier) {
        DrivetrainPublisher.inFieldCentricSupplier = inFieldCentricSupplier;
    }

    public static void setXVelocitySupplier(DoubleSupplier xVelocitySupplier) {
        DrivetrainPublisher.xVelocitySupplier = xVelocitySupplier;
    }

    public static void setXVelocitySupplier(DoubleSupplier xVelocitySupplier, boolean onlyX) {
        DrivetrainPublisher.xVelocitySupplier = xVelocitySupplier;

        if (onlyX) {
            DrivetrainPublisher.yVelocitySupplier = () -> 0;
            DrivetrainPublisher.thetaVelocitySupplier = () -> 0;
        }
    }

    public static void setYVelocitySupplier(DoubleSupplier yVelocitySupplier) {
        DrivetrainPublisher.yVelocitySupplier = yVelocitySupplier;
    }

    public static void setYVelocitySupplier(DoubleSupplier yVelocitySupplier, boolean onlyY) {
        DrivetrainPublisher.yVelocitySupplier = yVelocitySupplier;

        if (onlyY) {
            DrivetrainPublisher.xVelocitySupplier = () -> 0;
            DrivetrainPublisher.thetaVelocitySupplier = () -> 0;
        }
    }

    public static void setThetaVelocitySupplier(DoubleSupplier thetaVelocitySupplier) {
        DrivetrainPublisher.thetaVelocitySupplier = thetaVelocitySupplier;
    }

    public static void setThetaVelocitySupplier(DoubleSupplier thetaVelocitySupplier, boolean onlyTheta) {
        DrivetrainPublisher.thetaVelocitySupplier = thetaVelocitySupplier;

        if (onlyTheta) {
            DrivetrainPublisher.xVelocitySupplier = () -> 0;
            DrivetrainPublisher.yVelocitySupplier = () -> 0;
        }
    }

    public static void setSuppliers(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, DoubleSupplier thetaVelocitySupplier) {
        DrivetrainPublisher.xVelocitySupplier = xVelocitySupplier;
        DrivetrainPublisher.yVelocitySupplier = yVelocitySupplier;
        DrivetrainPublisher.thetaVelocitySupplier = thetaVelocitySupplier;
    }
    public static void setSuppliers(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, DoubleSupplier thetaVelocitySupplier, BooleanSupplier inFieldCentricSupplier) {
        DrivetrainPublisher.xVelocitySupplier = xVelocitySupplier;
        DrivetrainPublisher.yVelocitySupplier = yVelocitySupplier;
        DrivetrainPublisher.thetaVelocitySupplier = thetaVelocitySupplier;
        DrivetrainPublisher.inFieldCentricSupplier = inFieldCentricSupplier;
    }

    public static void setAcceptInputsSupplier(BooleanSupplier acceptInputsSupplier) {
        DrivetrainPublisher.acceptInputsSupplier = acceptInputsSupplier;
    }

    @FunctionalInterface
    public interface ReceiveSupplierValues {
        void receive(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, DoubleSupplier thetaVelocitySupplier, BooleanSupplier inFieldCentric, BooleanSupplier acceptInputsSupplier);
    }

    public static void updateDrivetrain(ReceiveSupplierValues driveFunction) {
        driveFunction.receive(DrivetrainPublisher.xVelocitySupplier, DrivetrainPublisher.yVelocitySupplier, DrivetrainPublisher.thetaVelocitySupplier, DrivetrainPublisher.inFieldCentricSupplier, DrivetrainPublisher.acceptInputsSupplier);
    }
}