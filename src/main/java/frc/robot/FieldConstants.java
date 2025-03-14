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

    // Reef pole robot positions

    public static final Pose2d REEF_A = new Pose2d(3.213, 4.186, Rotation2d.fromDegrees(0.0));
    public static final Pose2d REEF_B = new Pose2d(3.213, 3.856, Rotation2d.fromDegrees(0.0));
    public static final Pose2d REEF_C = new Pose2d(3.708, 2.998, Rotation2d.fromDegrees(60.0));
    public static final Pose2d REEF_D = new Pose2d(3.994, 2.833, Rotation2d.fromDegrees(60.0));
    public static final Pose2d REEF_E = new Pose2d(4.985, 2.833, Rotation2d.fromDegrees(120.0));
    public static final Pose2d REEF_F = new Pose2d(5.271, 2.998, Rotation2d.fromDegrees(120.0));
    public static final Pose2d REEF_G = new Pose2d(5.766, 3.856, Rotation2d.fromDegrees(180.0));
    public static final Pose2d REEF_H = new Pose2d(5.766, 4.186, Rotation2d.fromDegrees(180.0));
    public static final Pose2d REEF_I = new Pose2d(5.271, 5.044, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d REEF_J = new Pose2d(4.985, 5.209, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d REEF_K = new Pose2d(3.994, 5.209, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d REEF_L = new Pose2d(3.708, 5.044, Rotation2d.fromDegrees(-60.0));

    // Algae robot positions - these positions are 4.0 inches short of the wall

    public static final Pose2d REEF_ALGAE_AB = new Pose2d(3.111, 4.021, Rotation2d.fromDegrees(0.0));
    public static final Pose2d REEF_ALGAE_CD = new Pose2d(3.800, 2.827, Rotation2d.fromDegrees(60.0));
    public static final Pose2d REEF_ALGAE_EF = new Pose2d(5.178, 2.827, Rotation2d.fromDegrees(120.0));
    public static final Pose2d REEF_ALGAE_GH = new Pose2d(5.867, 4.021, Rotation2d.fromDegrees(180.0));
    public static final Pose2d REEF_ALGAE_IJ = new Pose2d(5.178, 5.214, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d REEF_ALGAE_KL = new Pose2d(3.800, 5.214, Rotation2d.fromDegrees(-60.0));

    // Coral Slot robot locations - these positions push 2.0 inches into the wall

    public static final Pose2d SOURCE_1_SLOT1 = new Pose2d(0.425, 1.452, Rotation2d.fromDegrees(54.0));
    public static final Pose2d SOURCE_1_SLOT2 = new Pose2d(0.589, 1.332, Rotation2d.fromDegrees(54.0));
    public static final Pose2d SOURCE_1_SLOT3 = new Pose2d(0.754, 1.213, Rotation2d.fromDegrees(54.0));
    public static final Pose2d SOURCE_1_SLOT4 = new Pose2d(0.918, 1.093, Rotation2d.fromDegrees(54.0));
    public static final Pose2d SOURCE_1_SLOT5 = new Pose2d(1.083, 0.974, Rotation2d.fromDegrees(54.0));
    public static final Pose2d SOURCE_1_SLOT6 = new Pose2d(1.247, 0.854, Rotation2d.fromDegrees(54.0));
    public static final Pose2d SOURCE_1_SLOT7 = new Pose2d(1.411, 0.735, Rotation2d.fromDegrees(54.0));
    public static final Pose2d SOURCE_1_SLOT8 = new Pose2d(1.576, 0.616, Rotation2d.fromDegrees(54.0));
    public static final Pose2d SOURCE_1_SLOT9 = new Pose2d(1.740, 0.496, Rotation2d.fromDegrees(54.0));

    public static final Pose2d SOURCE_2_SLOT1 = new Pose2d(0.425, 6.590, Rotation2d.fromDegrees(-54.0));
    public static final Pose2d SOURCE_2_SLOT2 = new Pose2d(0.589, 6.710, Rotation2d.fromDegrees(-54.0));
    public static final Pose2d SOURCE_2_SLOT3 = new Pose2d(0.754, 6.829, Rotation2d.fromDegrees(-54.0));
    public static final Pose2d SOURCE_2_SLOT4 = new Pose2d(0.918, 6.949, Rotation2d.fromDegrees(-54.0));
    public static final Pose2d SOURCE_2_SLOT5 = new Pose2d(1.083, 7.068, Rotation2d.fromDegrees(-54.0));
    public static final Pose2d SOURCE_2_SLOT6 = new Pose2d(1.247, 7.188, Rotation2d.fromDegrees(-54.0));
    public static final Pose2d SOURCE_2_SLOT7 = new Pose2d(1.411, 7.307, Rotation2d.fromDegrees(-54.0));
    public static final Pose2d SOURCE_2_SLOT8 = new Pose2d(1.576, 7.426, Rotation2d.fromDegrees(-54.0));
    public static final Pose2d SOURCE_2_SLOT9 = new Pose2d(1.740, 7.546, Rotation2d.fromDegrees(-54.0));

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
