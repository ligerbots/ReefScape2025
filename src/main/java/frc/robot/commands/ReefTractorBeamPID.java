// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefTractorBeamPID extends Command {
    /** Creates a new ReefTractorBeamPID. */
    private final DriveTrain m_driveTrain;
    private final double m_P = 1;
    private final double m_I = 0;
    private final double m_D = 0;
    private final PIDController m_pid = new PIDController(m_P, m_I, m_D);
    private double m_tolorence;

    public ReefTractorBeamPID(DriveTrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        m_driveTrain = drivetrain;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d currentPose = m_driveTrain.getPose();
        Pose2d nearestPole = FieldConstants.flipPose(FieldConstants.flipPose(currentPose).nearest(FieldConstants.REEF_SCORING_LOCATIONS));

        double translationX = -m_pid.calculate(currentPose.getX(), nearestPole.getX());
        double translationY = -m_pid.calculate(currentPose.getY(), nearestPole.getY());

        m_tolorence = Math.abs(translationY) + Math.abs(translationX);

        m_driveTrain.driveWithHeading(translationX, translationY, nearestPole.getRotation().getRadians());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_tolorence < 0.01 && 0 != m_tolorence;
    }
}
