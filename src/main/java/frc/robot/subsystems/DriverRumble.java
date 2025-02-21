// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;

public class DriverRumble extends SubsystemBase {
    /** Creates a new DriverRumble. */
    
    //Note: We can add source pickup locations to the list below so we get feedback for source alignment as well as the reef.
    static final List<Pose2d> REEF_SCORING_LOCATIONS = List.of(FieldConstants.REEF_A, FieldConstants.REEF_B,
            FieldConstants.REEF_C,
            FieldConstants.REEF_D, FieldConstants.REEF_E, FieldConstants.REEF_F,
            FieldConstants.REEF_G, FieldConstants.REEF_H, FieldConstants.REEF_I,
            FieldConstants.REEF_J, FieldConstants.REEF_K, FieldConstants.REEF_L);
    
    // Tolerance for lateral offset (meters) within which no rumble is applied.
    private final double REEF_OFFSET_TOLERANCE_METER = Units.inchesToMeters(2.0);
    // Maximum lateral offset (meters) that corresponds to full rumble intensity.
    // private final double METER_TO_TRIGGER = 0.5;
    
    // barge
    private final double BARGE_LINE_BLUE = 7.5;  // TODO a guess
    private final double BARGE_TARGET_TOLERANCE_METER = Units.inchesToMeters(3);

    private final double RUMBLE_INTENSITY = 1; 

    private final XboxController m_xbox;
    private final Supplier<Pose2d> m_robotPositionSupplier;
    private final BooleanSupplier m_hasCoral;
    private final BooleanSupplier m_hasAlgae;

    public DriverRumble(XboxController xbox, Supplier<Pose2d> positionSupplier, BooleanSupplier hasCoral, BooleanSupplier hasAlgae) {
        m_xbox = xbox;
        m_robotPositionSupplier = positionSupplier;
        m_hasCoral = hasCoral;
        m_hasAlgae = hasAlgae;
    }
    
    @Override
    public void periodic() {
        Pose2d robotPose = m_robotPositionSupplier.get();
        boolean rumble = false;

        if (m_hasCoral.getAsBoolean()) {
            // See if we are aligned with a Reef pole
            Pose2d targetPose = getClosestScoringLocation();
            
            // Relative pose rotated to the target pose
            Pose2d relativePose = robotPose.relativeTo(targetPose); 
            // final double distance = relativePose.getTranslation().getNorm();
            
            // Calculate lateral offset in meters (positive means left, negative means right)
            double lateralOffset = relativePose.getY();
            rumble = Math.abs(lateralOffset) < REEF_OFFSET_TOLERANCE_METER;
        }
        else if (m_hasAlgae.getAsBoolean()) {
            // check if correct place for a Barge shot
            double bargeLine = BARGE_LINE_BLUE;
            if (FieldConstants.isRedAlliance())
                bargeLine = FieldConstants.FIELD_LENGTH - bargeLine;
            rumble = Math.abs(robotPose.getX() - bargeLine) < BARGE_TARGET_TOLERANCE_METER;
        }

        m_xbox.setRumble(RumbleType.kBothRumble, rumble ? RUMBLE_INTENSITY : 0);

        // final double signedRumbleValue = getRumble(lateralOffset);
        
        // if (distance < METER_TO_TRIGGER && signedRumbleValue != 0) {
        //     System.out.println("Within range, activating rumble");
        //     System.out.println("Lateral Offset (m): " + lateralOffset);
        //     System.out.println("Rumble Value: " + signedRumbleValue);
        //     if (signedRumbleValue > 0) {
        //         m_xbox.setRumble(RumbleType.kLeftRumble, RUMBLE_INTENSITY);
        //         m_xbox.setRumble(RumbleType.kRightRumble, 0);
        //         System.out.println("Rumble Left");
        //     } else if (signedRumbleValue < 0) {
        //         m_xbox.setRumble(RumbleType.kRightRumble, RUMBLE_INTENSITY);
        //         m_xbox.setRumble(RumbleType.kLeftRumble, 0);
        //         System.out.println("Rumble Right");
        //     }
        // } else {
        //     m_xbox.setRumble(RumbleType.kBothRumble, 0);
        // }
    }
    
    // double getRumble(double lateralOffset) {
    //     // If the lateral offset is within tolerance, no rumble is needed.
    //     if (Math.abs(lateralOffset) < LATERAL_OFFSET_TOLERANCE_METER) {
    //         return 0;
    //     }
    //     // Scale the lateral offset to a value between -1 and 1.
    //     double strength = lateralOffset / METER_TO_TRIGGER;
    //     // Clamp the strength between -1 and 1.
    //     return MathUtil.clamp(strength, -1, 1);
    // }
    
    public Pose2d getClosestScoringLocation() {
        final Pose2d robotPose = m_robotPositionSupplier.get();
        
        Pose2d currentPose = FieldConstants.flipPose(robotPose);
        // System.out.println("Current Pose: " + currentPose);
        
        Pose2d nearestLocation = FieldConstants.flipPose(currentPose.nearest(REEF_SCORING_LOCATIONS));
        // System.out.println("Nearest Pole: " + nearestPole);
        return nearestLocation;
    }
}
