// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoReefAlignPID extends Command {
    /** Creates a new ReefTractorBeamPID. */
    private final DriveTrain m_driveTrain;
    private final CommandXboxController m_drivController;
    private final double m_P = 0.25;
    private final double m_I = 0;
    private final double m_D = 0;
    private final PIDController m_pid = new PIDController(m_P, m_I, m_D);
    private double m_tolorence;
    private Pose2d m_currentPose;
    private Pose2d m_targetPose;
    private long m_startTime;
    private BooleanSupplier m_hasCoral;

    public AutoReefAlignPID(DriveTrain drivetrain, CommandXboxController driveController, BooleanSupplier hasCoral) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements();
        m_driveTrain = drivetrain;
        m_drivController = driveController;
        m_hasCoral = hasCoral;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_currentPose = m_driveTrain.getPose();
        m_startTime = System.currentTimeMillis();
        m_pid.setSetpoint(0); //Using offset with the PID, may need to flip but zero is simpler
        m_targetPose = FieldConstants.flipPose(FieldConstants.flipPose(m_currentPose).nearest(FieldConstants.REEF_SCORING_LOCATIONS));
        // m_targetPose = new Pose2d(9.31, 0.80, Rotation2d.fromDegrees(-120));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_currentPose = m_driveTrain.getPose();
        //Yes, this is nesssary to allow target changing. This should probly be a subsystem
        // m_targetPose = FieldConstants.flipPose(FieldConstants.flipPose(m_currentPose).nearest(FieldConstants.REEF_SCORING_LOCATIONS));

        Pose2d currentPoseRelitiveToGoal = m_currentPose.relativeTo(m_targetPose);

        if (m_targetPose.getTranslation().getDistance(m_currentPose.getTranslation()) > 0.5) {
            return;
        }
        // double currentDistance = m_currentPose.getTranslation().getNorm();

        double magnitude = m_pid.calculate(currentPoseRelitiveToGoal.getY());
        Translation2d translation = new Translation2d(magnitude, Rotation2d.fromDegrees(m_targetPose.getRotation().getDegrees() - 90));

        // m_tolorence = Math.abs(translation.getX()) + Math.abs(magnitude);

        m_driveTrain.driveWithHeading(translation.getX()-m_drivController.getLeftY(), translation.getY()-m_drivController.getLeftX(), m_targetPose.getRotation().getRadians());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Done! \nCurrent pose:" + m_currentPose + "\nGoal pose:" + m_targetPose + "\nTime: " + (System.currentTimeMillis() - m_startTime) / 1000.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // boolean isWithinTolorence = m_tolorence < 0.001;
        // return isWithinTolorence;
        return false; // !m_hasCoral.getAsBoolean();
    }
}
