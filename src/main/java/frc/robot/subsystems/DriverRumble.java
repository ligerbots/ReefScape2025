// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;

public class DriverRumble extends SubsystemBase {    
    
    private static final double CLIMB_LOCATION_FROM_CENTER = Units.inchesToMeters(6);

    // Tolerance for lateral offset (meters) within which no rumble is applied.
    private final double REEF_OFFSET_TOLERANCE_METER = Units.inchesToMeters(1.0);
    // distance to position
    private final double METER_TO_TRIGGER = 0.25;
    
    // barge
    private final double BARGE_LINE_BLUE = 7.5; // meters
    private final double BARGE_TARGET_TOLERANCE_METER = Units.inchesToMeters(3);

    private final double RUMBLE_INTENSITY = 1; 

    private final XboxController m_xbox;
    private final Supplier<Pose2d> m_robotPositionSupplier;
    private final BooleanSupplier m_hasCoral;
    private final BooleanSupplier m_hasAlgae;
    private final BooleanSupplier m_climberDeployed;

    public DriverRumble(XboxController xbox, Supplier<Pose2d> positionSupplier, 
            BooleanSupplier hasCoral, BooleanSupplier hasAlgae, BooleanSupplier climberDeployed) {
        m_xbox = xbox;
        m_robotPositionSupplier = positionSupplier;
        m_hasCoral = hasCoral;
        m_hasAlgae = hasAlgae;
        m_climberDeployed = climberDeployed;
    }
    
    @Override
    public void periodic() {
        Pose2d robotPose = m_robotPositionSupplier.get();
        boolean rumble = false;
        double rumbleValue = 0;

        if (m_hasCoral.getAsBoolean()) {
            // See if we are aligned with a Reef pole
            Pose2d targetPose = getClosestScoringLocation();
            
            // Relative pose rotated to the target pose
            Pose2d relativePose = robotPose.relativeTo(targetPose); 
            final double distance = relativePose.getTranslation().getNorm();
            
            // Calculate lateral offset in meters (positive means left, negative means right)
            rumbleValue = relativePose.getY();
            rumble = distance < METER_TO_TRIGGER && Math.abs(rumbleValue) < REEF_OFFSET_TOLERANCE_METER;
        }
        else if (m_hasAlgae.getAsBoolean()) {
            // check if correct place for a Barge shot
            double bargeLine = BARGE_LINE_BLUE;
            if (FieldConstants.isRedAlliance())
                bargeLine = FieldConstants.FIELD_LENGTH - bargeLine;
            rumbleValue = robotPose.getX() - bargeLine;
            rumble = Math.abs(rumbleValue) < BARGE_TARGET_TOLERANCE_METER;
        }
        else if (m_climberDeployed.getAsBoolean()) {
            // check if correct place for a Barge shot
            double xMax = 0.5 * FieldConstants.FIELD_LENGTH;
            double xMin = xMax - CLIMB_LOCATION_FROM_CENTER;

            double xBlue = robotPose.getX();
            if (FieldConstants.isRedAlliance())
                xBlue = FieldConstants.FIELD_LENGTH - xBlue;
            rumbleValue = xBlue - xMin;
            rumble = xBlue >= xMin && xBlue <= xMax;
        }

        m_xbox.setRumble(RumbleType.kBothRumble, rumble ? RUMBLE_INTENSITY : 0);

        SmartDashboard.putBoolean("rumble", rumble);
        SmartDashboard.putNumber("rumbleValue", rumbleValue);

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
        
        Pose2d nearestLocation = FieldConstants.flipPose(currentPose.nearest(FieldConstants.REEF_SCORING_LOCATIONS));
        // System.out.println("Nearest Pole: " + nearestPole);
        return nearestLocation;
    }
}
