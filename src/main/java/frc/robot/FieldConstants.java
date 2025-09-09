package frc.robot;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {

    public static final double FIELD_LENGTH = FlippingUtil.fieldSizeX;
    public static final double FIELD_WIDTH = FlippingUtil.fieldSizeY;

    public static final Pose2d ROBOT_START_1 = new Pose2d(7.29, 1.64, Rotation2d.fromDegrees(150.6));
    public static final Pose2d ROBOT_START_2 = new Pose2d(7.23, 4.20, Rotation2d.fromDegrees(180));
    public static final Pose2d ROBOT_START_3 = new Pose2d(7.29, 6.75, Rotation2d.fromDegrees(-149.9));  

    // Reef pole robot positions

    public static final Pose2d REEF_A = new Pose2d(3.213, 4.186, Rotation2d.fromDegrees(180));
    public static final Pose2d REEF_B = new Pose2d(3.213, 3.856, Rotation2d.fromDegrees(180));
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

    
    public static Pose2d ALT_REEF_A;
    public static Pose2d ALT_REEF_B;
    public static Pose2d ALT_REEF_C;
    public static Pose2d ALT_REEF_D;
    public static Pose2d ALT_REEF_E;
    public static Pose2d ALT_REEF_F;
    public static Pose2d ALT_REEF_G;
    public static Pose2d ALT_REEF_H;
    public static Pose2d ALT_REEF_I;
    public static Pose2d ALT_REEF_J;
    public static Pose2d ALT_REEF_K;
    public static Pose2d ALT_REEF_L;

    // Algae robot positions - these positions are 1.0 inches short of the wall

    public static final Pose2d REEF_ALGAE_AB = new Pose2d(3.188, 4.021, Rotation2d.fromDegrees(0.0));
    public static final Pose2d REEF_ALGAE_CD = new Pose2d(3.839, 2.893, Rotation2d.fromDegrees(60.0));
    public static final Pose2d REEF_ALGAE_EF = new Pose2d(5.140, 2.893, Rotation2d.fromDegrees(120.0));
    public static final Pose2d REEF_ALGAE_GH = new Pose2d(5.791, 4.021, Rotation2d.fromDegrees(180.0));
    public static final Pose2d REEF_ALGAE_IJ = new Pose2d(5.140, 5.148, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d REEF_ALGAE_KL = new Pose2d(3.839, 5.148, Rotation2d.fromDegrees(-60.0));
    
    public static Pose2d ALT_REEF_ALGAE_AB;
    public static Pose2d ALT_REEF_ALGAE_CD;
    public static Pose2d ALT_REEF_ALGAE_EF;
    public static Pose2d ALT_REEF_ALGAE_GH;
    public static Pose2d ALT_REEF_ALGAE_IJ;
    public static Pose2d ALT_REEF_ALGAE_KL;

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
    

    
        //TODO get real center of reef 
    public static final Pose2d REEF_CENTER = new Pose2d(FIELD_WIDTH/2 ,Units.feetToMeters(12.0), Rotation2d.fromDegrees(0)); 

     // Dynamic lists - filled in init()
     public static List<Pose2d> REEF_SCORING_LOCATIONS;
     public static List<Pose2d> ALT_REEF_SCORING_LOCATIONS;
 
     // Call this once from robotInit()
     public static void init() {
         ALT_REEF_A = returnAltReefPose(REEF_A, REEF_B);
         ALT_REEF_B = returnAltReefPose(REEF_B, REEF_A);
         ALT_REEF_C = returnAltReefPose(REEF_C, REEF_D);
         ALT_REEF_D = returnAltReefPose(REEF_D, REEF_C);
         ALT_REEF_E = returnAltReefPose(REEF_E, REEF_F);
         ALT_REEF_F = returnAltReefPose(REEF_F, REEF_E);
         ALT_REEF_G = returnAltReefPose(REEF_G, REEF_H);
         ALT_REEF_H = returnAltReefPose(REEF_H, REEF_G);
         ALT_REEF_I = returnAltReefPose(REEF_I, REEF_J);
         ALT_REEF_J = returnAltReefPose(REEF_J, REEF_I);
         ALT_REEF_K = returnAltReefPose(REEF_K, REEF_L);
         ALT_REEF_L = returnAltReefPose(REEF_L, REEF_K);
 
         ALT_REEF_ALGAE_AB = REEF_ALGAE_AB;
         ALT_REEF_ALGAE_CD = REEF_ALGAE_CD;
         ALT_REEF_ALGAE_EF = REEF_ALGAE_EF;
         ALT_REEF_ALGAE_GH = REEF_ALGAE_GH;
         ALT_REEF_ALGAE_IJ = REEF_ALGAE_IJ;
         ALT_REEF_ALGAE_KL = REEF_ALGAE_KL;
 
         REEF_SCORING_LOCATIONS = List.of(
             REEF_A, REEF_B, REEF_C, REEF_D, REEF_E, REEF_F,
             REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L
         );
 
         ALT_REEF_SCORING_LOCATIONS = List.of(
             ALT_REEF_A, ALT_REEF_B, ALT_REEF_C, ALT_REEF_D, ALT_REEF_E, ALT_REEF_F,
             ALT_REEF_G, ALT_REEF_H, ALT_REEF_I, ALT_REEF_J, ALT_REEF_K, ALT_REEF_L
         );
     }

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

    public static Pose2d returnAltReefPose(Pose2d wantedPos, Pose2d refPos){
        double altshift = Units.inchesToMeters(4.0); //how far from the wall we want 
        
        double wantedX = wantedPos.getX();
        double wantedY = wantedPos.getY();
        double refX = refPos.getX();
        double refY = refPos.getY();

        Translation2d midpoint = new Translation2d((wantedX+refX)/2, (wantedY+refY)/2);
        Translation2d dirVector = new Translation2d(refX-wantedX, refY-wantedY);

        Translation2d perpVector1 = new Translation2d(-dirVector.getX(), dirVector.getY());
        Translation2d perpVector2 = new Translation2d(dirVector.getX(), -dirVector.getY());
        //get both because we dont know which one is pointing out. 
        Translation2d outwardNormal;
        if( dot(perpVector1, midpoint.minus(REEF_CENTER.getTranslation())) > 0 ){
            outwardNormal = perpVector1;
        }else{
            outwardNormal = perpVector2;
        }

        Translation2d shiftVector = outwardNormal.times(altshift/outwardNormal.getNorm());

        Translation2d shiftedPos = wantedPos.getTranslation().plus(shiftVector);

        return new Pose2d(shiftedPos, wantedPos.getRotation().plus(new Rotation2d(Math.toRadians(180))));
    }

    private static double dot(Translation2d v1, Translation2d v2){
        return v1.getX()*v2.getX()+v1.getY()*v2.getY();
    }
}


