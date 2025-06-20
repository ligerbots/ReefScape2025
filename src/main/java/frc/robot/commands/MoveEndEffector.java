// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;

public class MoveEndEffector extends Command {
    EndEffectorPivot m_pivot;
    Elevator m_elevator;
    // boolean m_cancel;  // TODO some way to cancel the motion?
    
    Rotation2d m_desiredAngle;
    double m_desiredHeight;
    Constants.Position m_position;
    Timer m_commandTimeout = new Timer();
    double m_timeoutDelay;

    private static final double DEFAULT_TIMEOUT = 2.0;
    
    private static final double L1_ANGLE = 310.0;
    public static final double L1_HEIGHT = Units.inchesToMeters(2.0);
    private static final double L2_ANGLE = 280;
    private static final double L2_HEIGHT = Units.inchesToMeters(1.5);
    private static final double L3_ANGLE = 275;
    private static final double L3_HEIGHT = Units.inchesToMeters(17.0);
    private static final double L4_ANGLE = 318.0;
    private static final double L4_HEIGHT = Units.inchesToMeters(58.0);
    
    private static final double STOW_ANGLE = 138.0;
    private static final double STOW_HEIGHT = 0.0;
    
    private static final double BARGE_HEIGHT = Units.inchesToMeters(62.0);
    private static final double BARGE_ANGLE = 300.0;
    private static final double ALT_BARGE_HEIGHT = Units.inchesToMeters(63.0);  //TODO FIX ME
    private static final double ALT_BARGE_ANGLE = 200.0;
    
    private static final double FRONT_INTAKE_HEIGHT = 0;
    private static final double FRONT_INTAKE_ANGLE = 245;
    private static final double BACK_INTAKE_HEIGHT = Units.inchesToMeters(4.25);
    private static final double BACK_INTAKE_ANGLE = 127.5;
    
    
    private static final double L2_ALGAE_HEIGHT= Units.inchesToMeters(3.5);
    private static final double L2_ALGAE_ANGLE = 325.0;
    
    private static final double L3_ALGAE_HEIGHT= Units.inchesToMeters(25);
    private static final double L3_ALGAE_ANGLE = 335.0;    

    private static final double PROCESSOR_HEIGHT = Units.inchesToMeters(0);
    private static final double PROCESSOR_ANGLE = 140;

    private static final double CLIMB_ANGLE = 296.0;
    private static final double CLIMB_HEIGHT = 0.0;

    // support delaying the elevator motion for a little bit
    // allows the pivot to start moving out of the way
    private static final double ELEVATOR_DELAY_HEIGHT = L4_HEIGHT - 0.1;
    private static final double ELEVATOR_DELAY_TIME = 0.1;
    private Timer m_elevatorTimer = new Timer();
    private boolean m_elevatorSet = false;

    private static final HashMap<Position, Pair<Double, Double>> POSITIONS = new HashMap<Position, Pair<Double, Double>>() {
        {
            put(Position.L1, new Pair<>(L1_HEIGHT, L1_ANGLE));
            put(Position.L2, new Pair<>(L2_HEIGHT, L2_ANGLE));
            put(Position.L3, new Pair<>(L3_HEIGHT, L3_ANGLE));
            put(Position.L4, new Pair<>(L4_HEIGHT, L4_ANGLE));
            put(Position.BARGE, new Pair<>(ALT_BARGE_HEIGHT, ALT_BARGE_ANGLE));
            put(Position.FRONT_INTAKE, new Pair<>(FRONT_INTAKE_HEIGHT, FRONT_INTAKE_ANGLE));
            put(Position.BACK_INTAKE, new Pair<>(BACK_INTAKE_HEIGHT, BACK_INTAKE_ANGLE));
            put(Position.L2_ALGAE, new Pair<>(L2_ALGAE_HEIGHT, L2_ALGAE_ANGLE));
            put(Position.L3_ALGAE, new Pair<>(L3_ALGAE_HEIGHT, L3_ALGAE_ANGLE));
            put(Position.STOW, new Pair<>(STOW_HEIGHT, STOW_ANGLE));
            put(Position.PROCESSOR, new Pair<>(PROCESSOR_HEIGHT, PROCESSOR_ANGLE));
            put(Position.CLIMB, new Pair<Double,Double>(CLIMB_HEIGHT, CLIMB_ANGLE));
        }
    };
    
    public MoveEndEffector(Constants.Position position, Elevator elevator, EndEffectorPivot pivot) {
        this(position, elevator, pivot, DEFAULT_TIMEOUT);
    }

    public MoveEndEffector(Constants.Position position, Elevator elevator, EndEffectorPivot pivot, double timeout) {
        m_pivot = pivot;
        m_elevator = elevator;
        m_position = position;
        m_timeoutDelay = timeout;

        Pair<Double, Double> desiredPos = POSITIONS.get(position);
        m_desiredHeight = desiredPos.getFirst();
        m_desiredAngle = Rotation2d.fromDegrees(desiredPos.getSecond());
        
        // Require the elevator and pivot, since we are waiting for them to reach goal
        addRequirements(elevator, pivot);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_pivot.setAngle(m_desiredAngle);

        // figure out whether to set the elevator immediately, or delay a bit
        m_elevatorSet = true;
        double height = m_elevator.getHeight();
        if (height > ELEVATOR_DELAY_HEIGHT) {
            // elevator is high. If the pivot only moves a bit, don't delay.
            Rotation2d angle = m_pivot.getAngle();
            m_elevatorSet = Math.abs(angle.minus(m_desiredAngle).getDegrees()) < 90.0;
        }

        if (m_elevatorSet) {
            m_elevator.setHeight(m_desiredHeight);
        } else {
            m_elevatorTimer.restart();
        }

        m_commandTimeout.restart();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!m_elevatorSet && m_elevatorTimer.hasElapsed(ELEVATOR_DELAY_TIME)) {
            m_elevator.setHeight(m_desiredHeight);
            m_elevatorSet = true;
        }
    }
    
    // // Called once the command ends or is interrupted.
    // @Override
    // public void end(boolean interrupted) {}
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (m_elevatorSet && m_elevator.lengthWithinTolerance() && m_pivot.angleWithinTolerance())
                || m_commandTimeout.hasElapsed(m_timeoutDelay);
    }
}
