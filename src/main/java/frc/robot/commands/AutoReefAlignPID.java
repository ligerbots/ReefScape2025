// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private Translation2d m_compensationTranslation;
    private double POSE_TOLORENCE = 0.0254;
    private Pose2d m_currentPose;
    private Pose2d m_targetPose;
    // private Pose2d m_targetPoleEnd;
    private long m_startTime;
    private double TRIGGER_DISTANCE = 1;

    public AutoReefAlignPID(DriveTrain drivetrain, CommandXboxController driveController) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        m_driveTrain = drivetrain;
        m_drivController = driveController;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_currentPose = m_driveTrain.getPose(); // Leave this, this is used in init.
        m_startTime = System.currentTimeMillis();
        m_pid.setSetpoint(0);
        m_targetPose = FieldConstants
                .flipPose(FieldConstants.flipPose(m_currentPose).nearest(FieldConstants.REEF_SCORING_LOCATIONS));
        // m_targetPoleEnd = FieldConstants
        // .flipPose(FieldConstants.flipPose(m_currentPose).nearest(FieldConstants.REEF_POLE_END_LOCATIONS));

        System.out.println("Target Pose: " + m_targetPose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // TODO:
        // Cancel on timeout

        m_currentPose = m_driveTrain.getPose();
        Pose2d currentPoseRelitiveToGoal = m_currentPose.relativeTo(m_targetPose);
        if (currentPoseRelitiveToGoal.getTranslation().getNorm() > TRIGGER_DISTANCE) {
            return;
        }

        Rotation2d targetRotation = m_targetPose.getRotation();
        // This filters sideways motion relitive to the goal heading out.
        Translation2d driverTranslation = new Translation2d(m_drivController.getLeftX(), m_drivController.getLeftY());
        double driverMagnitude = ((targetRotation.getCos() * driverTranslation.getX())
                + (targetRotation.getSin() * driverTranslation.getY()));
        Translation2d filteredDriverTranslation = new Translation2d(
                -driverMagnitude * targetRotation.getCos(),
                -driverMagnitude * targetRotation.getSin());

        double compensationMagnitude = m_pid.calculate(currentPoseRelitiveToGoal.getY());
        m_compensationTranslation = new Translation2d(compensationMagnitude,
                Rotation2d.fromDegrees(targetRotation.getDegrees() - 90));

        double rotationRadians = targetRotation.getRadians();

        m_driveTrain.driveWithHeading(m_compensationTranslation.getX() - filteredDriverTranslation.getX(),
                m_compensationTranslation.getY() - filteredDriverTranslation.getY(),
                rotationRadians);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Done! \nCurrent pose:" + m_currentPose + "\nGoal pose:" + m_targetPose + "\nTime: "
                + (System.currentTimeMillis() - m_startTime) / 1000.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean isWithinTolorence = m_currentPose.minus(m_targetPose).getTranslation().getNorm() < POSE_TOLORENCE;
        // return isWithinTolorence;
        return isWithinTolorence; // !m_hasCoral.getAsBoolean();
    }
}
