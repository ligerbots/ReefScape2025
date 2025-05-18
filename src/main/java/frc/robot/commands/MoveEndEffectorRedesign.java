// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import org.javatuples.Triplet;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;
import frc.robot.subsystems.EndEffectorWrist;

public class MoveEndEffectorRedesign extends Command {
    EndEffectorPivot m_pivot;
    Elevator m_elevator;
    EndEffectorWrist m_wrist;
    // boolean m_cancel;  // TODO some way to cancel the motion?
    
    Rotation2d m_desiredPivotAngle;
    double m_desiredHeight;
    Rotation2d m_desiredWristAngle;
    Constants.Position m_position;
    Timer m_commandTimeout = new Timer();
    double m_timeoutDelay;

    private static final double DEFAULT_TIMEOUT = 2.0;
    
    private static final double L1_PIVOT_ANGLE = 310.0;
    public static final double L1_HEIGHT = Units.inchesToMeters(2.0);
    private static final double L1_WRIST_ANGLE = 310.0;


    private static final double L2_PIVOT_ANGLE = 270;
    public static final double L2_HEIGHT = Units.inchesToMeters(0);
    private static final double L2_WRIST_ANGLE = 0;

    private static final double L3_PIVOT_ANGLE = 270;
    public static final double L3_HEIGHT = Units.inchesToMeters(13.35);
    private static final double L3_WRIST_ANGLE = 0;

    private static final double L4_PIVOT_ANGLE = 270.0;
    public static final double L4_HEIGHT = Units.inchesToMeters(35.0);
    private static final double L4_WRIST_ANGLE = 0;

    private static final double L2_PIVOT_ANGLE_PREP = 245;
    public static final double L2_HEIGHT_PREP = Units.inchesToMeters(0);
    private static final double L2_WRIST_ANGLE_PREP = 0;

    private static final double L3_PIVOT_ANGLE_PREP = 245.0;
    public static final double L3_HEIGHT_PREP = Units.inchesToMeters(15.35);
    private static final double L3_WRIST_ANGLE_PREP = 0;

    private static final double L4_PIVOT_ANGLE_PREP = 256.0;
    private static final double L4_HEIGHT_PREP = Units.inchesToMeters(42.6);
    private static final double L4_WRIST_ANGLE_PREP = 0;

    private static final double TRANSFER_PIVOT_ANGLE_WAIT = 0;
    private static final double TRANSFER_HEIGHT_WAIT = Units.inchesToMeters(6.0);
    private static final double TRANSFER_WRIST_ANGLE_WAIT = 90;


    private static final double STOW_PIVOT_ANGLE = 180;
    private static final double STOW_HEIGHT = 0.0;
    private static final double STOW_WRIST_ANGLE = 0;

    private static final double TRANSFER_PIVOT_ANGLE = 0;
    private static final double TRANSFER_HEIGHT = 4.5;
    private static final double TRANSFER_WRIST_ANGLE = 90;


    private static final double BARGE_HEIGHT = Units.inchesToMeters(62.0);
    private static final double BARGE_PIVOT_ANGLE = 300.0;
    private static final double BARGE_WRIST_ANGLE = 0;

    private static final double ALT_BARGE_HEIGHT = Units.inchesToMeters(36.4);  //TODO FIX ME
    private static final double ALT_BARGE_PIVOT_ANGLE = 136;
    private static final double ALT_BARGE_WRIST_ANGLE = 0;
    
    private static final double FRONT_INTAKE_HEIGHT = 0;
    private static final double FRONT_INTAKE_PIVOT_ANGLE = 245;
    private static final double FRONT_INTAKE_WRIST_ANGLE = 0;


    private static final double BACK_INTAKE_HEIGHT = Units.inchesToMeters(4.5);
    private static final double BACK_INTAKE_PIVOT_ANGLE = 127.5;
    private static final double BACK_INTAKE_WRIST_ANGLE = 0;
    
    private static final double L2_ALGAE_HEIGHT= Units.inchesToMeters(3.1);
    private static final double L2_ALGAE_PIVOT_ANGLE = 270.0;
    private static final double L2_ALGAE_WRIST_ANGLE = 0;
    
    private static final double L3_ALGAE_HEIGHT= Units.inchesToMeters(19.0);
    private static final double L3_ALGAE_PIVOT_ANGLE = 270;    
    private static final double L3_ALGAE_WRITST_ANGLE = 0;

    private static final double PROCESSOR_HEIGHT = Units.inchesToMeters(0);
    private static final double PROCESSOR_PIVOT_ANGLE = 300;
    private static final double PROCESSOR_WIRST_ANGLE = 90;

    private static final double CLIMB_PIVOT_ANGLE = 0;
    private static final double CLIMB_HEIGHT = 4.5;
    private static final double CLIMB_WRIST_ANGLE = 0;



    // support delaying the elevator motion for a little bit
    // allows the pivot to start moving out of the way
    private static final double ELEVATOR_DELAY_HEIGHT = L4_HEIGHT - 0.1;
    private static final double ELEVATOR_DELAY_TIME = 0.1;
    private Timer m_elevatorTimer = new Timer();
    private boolean m_elevatorSet = false;

    private static final HashMap<Position, Triplet<Double, Double, Double>> POSITIONS = new HashMap<Position, Triplet<Double, Double, Double>>() {
        {
            put(Position.L1, new Triplet<Double, Double,Double>(L1_HEIGHT, L1_PIVOT_ANGLE, L1_WRIST_ANGLE));
            put(Position.L2, new Triplet<Double, Double,Double>(L2_HEIGHT, L2_PIVOT_ANGLE, L2_WRIST_ANGLE));
            put(Position.L3, new Triplet<Double, Double,Double>(L3_HEIGHT, L3_PIVOT_ANGLE, L3_WRIST_ANGLE));
            put(Position.L4, new Triplet<Double, Double,Double>(L4_HEIGHT, L4_PIVOT_ANGLE, L4_WRIST_ANGLE));
            put(Position.L2_PREP, new Triplet<Double, Double,Double>(L2_HEIGHT_PREP, L2_PIVOT_ANGLE_PREP, L2_WRIST_ANGLE_PREP));
            put(Position.L3_PREP, new Triplet<Double, Double,Double>(L3_HEIGHT_PREP, L3_PIVOT_ANGLE_PREP, L3_WRIST_ANGLE_PREP));
            put(Position.L4_PREP, new Triplet<Double, Double,Double>(L4_HEIGHT_PREP, L4_PIVOT_ANGLE_PREP, L4_WRIST_ANGLE_PREP));
            put(Position.BARGE, new Triplet<Double, Double,Double>(ALT_BARGE_HEIGHT, ALT_BARGE_PIVOT_ANGLE, ALT_BARGE_WRIST_ANGLE));
            put(Position.FRONT_INTAKE, new Triplet<Double, Double,Double>(FRONT_INTAKE_HEIGHT, FRONT_INTAKE_PIVOT_ANGLE, FRONT_INTAKE_WRIST_ANGLE));
            put(Position.BACK_INTAKE, new Triplet<Double, Double,Double>(BACK_INTAKE_HEIGHT, BACK_INTAKE_PIVOT_ANGLE, BACK_INTAKE_WRIST_ANGLE));
            put(Position.L2_ALGAE, new Triplet<Double, Double,Double>(L2_ALGAE_HEIGHT, L2_ALGAE_PIVOT_ANGLE, L2_ALGAE_WRIST_ANGLE));
            put(Position.L3_ALGAE, new Triplet<Double, Double,Double>(L3_ALGAE_HEIGHT, L3_ALGAE_PIVOT_ANGLE, L3_ALGAE_WRITST_ANGLE));
            put(Position.STOW, new Triplet<Double, Double,Double>(STOW_HEIGHT, STOW_PIVOT_ANGLE, STOW_WRIST_ANGLE));
            put(Position.PROCESSOR, new Triplet<Double, Double,Double>(PROCESSOR_HEIGHT, PROCESSOR_PIVOT_ANGLE, PROCESSOR_WIRST_ANGLE));
            put(Position.CLIMB, new Triplet<Double, Double,Double>(CLIMB_HEIGHT, CLIMB_PIVOT_ANGLE, CLIMB_WRIST_ANGLE));
            put(Position.TRANSFER, new Triplet<Double,Double,Double>(TRANSFER_HEIGHT, TRANSFER_PIVOT_ANGLE, TRANSFER_WRIST_ANGLE));
            put(Position.TRANSFER_WAIT, new Triplet<Double,Double,Double>(TRANSFER_HEIGHT_WAIT, TRANSFER_PIVOT_ANGLE_WAIT, TRANSFER_WRIST_ANGLE_WAIT));
        }
    };
    
    public MoveEndEffectorRedesign(Constants.Position position, Elevator elevator, EndEffectorPivot pivot, EndEffectorWrist wrist) {
        this(position, elevator, pivot, wrist, DEFAULT_TIMEOUT);
    }

    public MoveEndEffectorRedesign(Constants.Position position, Elevator elevator, EndEffectorPivot pivot, EndEffectorWrist wrist, double timeout) {
        m_pivot = pivot;
        m_elevator = elevator;
        m_position = position;
        m_wrist = wrist;
        m_timeoutDelay = timeout;

        Triplet<Double, Double, Double> desiredPos = POSITIONS.get(position);
        
        m_desiredHeight = desiredPos.getValue0();
        m_desiredPivotAngle = Rotation2d.fromDegrees(desiredPos.getValue1());
        m_desiredWristAngle = Rotation2d.fromDegrees(desiredPos.getValue2());
        
        // Require the elevator and pivot, since we are waiting for them to reach goal
        addRequirements(elevator, pivot, wrist);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_pivot.setAngle(m_desiredPivotAngle);
        m_wrist.setAngle(m_desiredWristAngle);
        m_elevator.setHeight(m_desiredHeight);

        // figure out whether to set the elevator immediately, or delay a bit
        // m_elevatorSet = true;
        // double height = m_elevator.getHeight();
        // if (height > ELEVATOR_DELAY_HEIGHT) {
        //     // elevator is high. If the pivot only moves a bit, don't delay.
        //     Rotation2d angle = m_pivot.getAngle();
        //     m_elevatorSet = Math.abs(angle.minus(m_desiredPivotAngle).getDegrees()) < 90.0;
        // }

        // if (m_elevatorSet) {
        //     m_elevator.setHeight(m_desiredHeight);
        // } else {
        //     m_elevatorTimer.restart();
        // }

        // m_commandTimeout.restart();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (!m_elevatorSet && m_elevatorTimer.hasElapsed(ELEVATOR_DELAY_TIME)) {
        //     m_elevator.setHeight(m_desiredHeight);
        //     m_elevatorSet = true;
        // }
    }
    
    // // Called once the command ends or is interrupted.
    // @Override
    // public void end(boolean interrupted) {}
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (m_elevator.lengthWithinTolerance() && m_pivot.angleWithinTolerance() && m_wrist.angleWithinTolerance())
                || m_commandTimeout.hasElapsed(m_timeoutDelay);
    }
}
