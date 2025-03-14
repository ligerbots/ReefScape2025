package frc.robot;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {

    public static final double FIELD_LENGTH = FlippingUtil.fieldSizeX;
    public static final double FIELD_WIDTH = FlippingUtil.fieldSizeY;

    public static final Pose2d ROBOT_START_1 = new Pose2d(7.29, 1.64, Rotation2d.fromDegrees(150.6));
    public static final Pose2d ROBOT_START_2 = new Pose2d(7.23, 4.20, Rotation2d.fromDegrees(180));
    public static final Pose2d ROBOT_START_3 = new Pose2d(7.29, 6.75, Rotation2d.fromDegrees(-149.9));  

    public static final Pose2d REEF_A = new Pose2d(3.21, 4.19, Rotation2d.fromDegrees(0.0));
    public static final Pose2d REEF_B = new Pose2d(3.21, 3.86, Rotation2d.fromDegrees(0.0));
    public static final Pose2d REEF_C = new Pose2d(3.71, 3.00, Rotation2d.fromDegrees(60.0));
    public static final Pose2d REEF_D = new Pose2d(3.99, 2.83, Rotation2d.fromDegrees(60.0));
    public static final Pose2d REEF_E = new Pose2d(4.98, 2.83, Rotation2d.fromDegrees(120.0));
    public static final Pose2d REEF_F = new Pose2d(5.27, 3.00, Rotation2d.fromDegrees(120.0));
    public static final Pose2d REEF_G = new Pose2d(5.77, 3.86, Rotation2d.fromDegrees(180.0));
    public static final Pose2d REEF_H = new Pose2d(5.77, 4.19, Rotation2d.fromDegrees(180.0));
    public static final Pose2d REEF_I = new Pose2d(5.27, 5.04, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d REEF_J = new Pose2d(4.98, 5.21, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d REEF_K = new Pose2d(3.99, 5.21, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d REEF_L = new Pose2d(3.71, 5.04, Rotation2d.fromDegrees(-60.0));

    public static final Pose2d SOURCE_1_IN = new Pose2d(0.64, 1.37, Rotation2d.fromDegrees(55.6));
    public static final Pose2d SOURCE_1_OUT = new Pose2d(1.65, 0.64, Rotation2d.fromDegrees(55.6));
    public static final Pose2d SOURCE_1_CENTER = new Pose2d(1.65, 0.64, Rotation2d.fromDegrees(55.6));

    public static final Pose2d SOURCE_2_IN = new Pose2d(0.63, 6.68, Rotation2d.fromDegrees(-52.7));
    public static final Pose2d SOURCE_2_CENTER = new Pose2d(1.17, 7.07, Rotation2d.fromDegrees(-52.7));
    public static final Pose2d SOURCE_2_OUT = new Pose2d(1.67, 7.41, Rotation2d.fromDegrees(-52.7));
    
    public static final List<Pose2d> REEF_SCORING_LOCATIONS = List.of(
        FieldConstants.REEF_A, FieldConstants.REEF_B,
        FieldConstants.REEF_C, FieldConstants.REEF_D, 
        FieldConstants.REEF_E, FieldConstants.REEF_F,
        FieldConstants.REEF_G, FieldConstants.REEF_H, 
        FieldConstants.REEF_I, FieldConstants.REEF_J, 
        FieldConstants.REEF_K, FieldConstants.REEF_L);

    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public static Pose2d flipPose(Pose2d pose) {
        // flip pose when red
        if (isRedAlliance()) {
            return FlippingUtil.flipFieldPose(pose);
        }

        // Blue or we don't know; return the original pose
        return pose;
    }

    public static Translation2d flipTranslation(Translation2d position) {
        // flip when red
        if (isRedAlliance()) {
            return FlippingUtil.flipFieldPosition(position);
        }

        // Blue or we don't know; return the original position
        return position;
    }

    public static Translation2d mirrorTranslation(Translation2d translation) {
        return new Translation2d(translation.getX(), FlippingUtil.fieldSizeY - translation.getY());
      }

    public static Pose2d mirrorPose(Pose2d pose) {
        return new Pose2d(mirrorTranslation(pose.getTranslation()), pose.getRotation().unaryMinus());
    }
}