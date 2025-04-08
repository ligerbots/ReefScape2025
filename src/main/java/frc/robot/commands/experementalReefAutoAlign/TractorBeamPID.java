// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.experementalReefAutoAlign;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TractorBeamPID extends Command {
    /** Creates a new ReefTractorBeamPID. */
    private final DriveTrain m_driveTrain;
    private final double m_P = 1.0;
    private final double m_I = 0;
    private final double m_D = 0;
    private final PIDController m_pid = new PIDController(m_P, m_I, m_D);
    private double m_distanceFromGoal;
    private Pose2d m_currentPose;
    private static final double DISTANCE_TOLORINCE_METERS = 1/39.37;

    private final Pose2d m_goalPose;

    public TractorBeamPID(DriveTrain drivetrain, Pose2d goalPose) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        m_driveTrain = drivetrain;
        m_goalPose = goalPose;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_currentPose = m_driveTrain.getPose();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_currentPose = m_driveTrain.getPose();

        double translationX = -m_pid.calculate(m_currentPose.getX(), m_goalPose.getX());
        double translationY = -m_pid.calculate(m_currentPose.getY(), m_goalPose.getY());

        m_distanceFromGoal = m_currentPose.getTranslation().minus(m_goalPose.getTranslation()).getNorm(); // get the distance to the goal

        m_driveTrain.driveWithHeading(translationX, translationY, m_goalPose.getRotation().getRadians());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean isWithinTolorence = m_distanceFromGoal <= DISTANCE_TOLORINCE_METERS;
        return isWithinTolorence;
    }
}