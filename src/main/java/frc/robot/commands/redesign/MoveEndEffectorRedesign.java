// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.redesign;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

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
    BooleanSupplier m_isAltMode;
    // boolean m_cancel;  // TODO some way to cancel the motion?
    
    Rotation2d m_desiredPivotAngle;
    double m_desiredHeight;
    Rotation2d m_desiredWristAngle;
    Constants.Position m_position;
    Constants.Position m_newPos;
    Timer m_commandTimeout = new Timer();
    double m_timeoutDelay;



    private static final double DEFAULT_TIMEOUT = 2.0;
    
    private static final double L1_PIVOT_ANGLE = 285.0;
    public static final double L1_HEIGHT = Units.inchesToMeters(2.0);
    private static final double L1_WRIST_ANGLE = 90;


    private static final double L2_PIVOT_ANGLE = 280;
    public static final double L2_HEIGHT = Units.inchesToMeters(0);
    private static final double L2_WRIST_ANGLE = 0;

    private static final double L3_PIVOT_ANGLE = 280;
    public static final double L3_HEIGHT = Units.inchesToMeters(13.35);
    private static final double L3_WRIST_ANGLE = 0;

    private static final double L4_PIVOT_ANGLE = 270.0;
    public static final double L4_HEIGHT = Units.inchesToMeters(32.0);
    private static final double L4_WRIST_ANGLE = 0;

    private static final double L2_PIVOT_ANGLE_PREP = 235;
    public static final double L2_HEIGHT_PREP = Units.inchesToMeters(1);
    private static final double L2_WRIST_ANGLE_PREP = 0;

    private static final double L3_PIVOT_ANGLE_PREP = 235.0;
    public static final double L3_HEIGHT_PREP = Units.inchesToMeters(16.35);
    private static final double L3_WRIST_ANGLE_PREP = 0;

    private static final double L4_PIVOT_ANGLE_PREP = 235;
    private static final double L4_HEIGHT_PREP = Units.inchesToMeters(37);
    private static final double L4_WRIST_ANGLE_PREP = 0;

    //TODO need to set alt values 

    private static final double L1_PIVOT_ANGLE_ALT = 285;
    public static final double L1_HEIGHT_ALT = Units.inchesToMeters(1);
    private static final double L1_WRIST_ANGLE_ALT = 90;

    private static final double L2_PIVOT_ANGLE_ALT = 50;
    public static final double L2_HEIGHT_ALT = Units.inchesToMeters(0);
    private static final double L2_WRIST_ANGLE_ALT = 0;

    private static final double L3_PIVOT_ANGLE_ALT = 100;
    public static final double L3_HEIGHT_ALT = Units.inchesToMeters(0);
    private static final double L3_WRIST_ANGLE_ALT = 0;

    private static final double L4_PIVOT_ANGLE_ALT = 90;
    public static final double L4_HEIGHT_ALT = Units.inchesToMeters(20);
    private static final double L4_WRIST_ANGLE_ALT = 0;

    private static final double L2_PIVOT_ANGLE_PREP_ALT = 85;
    public static final double L2_HEIGHT_PREP_ALT = Units.inchesToMeters(1);
    private static final double L2_WRIST_ANGLE_PREP_ALT = 0;

    private static final double L3_PIVOT_ANGLE_PREP_ALT = 110;
    public static final double L3_HEIGHT_PREP_ALT = Units.inchesToMeters(2);
    private static final double L3_WRIST_ANGLE_PREP_ALT = 0;

    private static final double L4_PIVOT_ANGLE_PREP_ALT = 90;
    private static final double L4_HEIGHT_PREP_ALT = Units.inchesToMeters(36);
    private static final double L4_WRIST_ANGLE_PREP_ALT = 0;

    private static final double TRANSFER_PIVOT_ANGLE_WAIT = 0;
    private static final double TRANSFER_HEIGHT_WAIT = Units.inchesToMeters(6.0);
    private static final double TRANSFER_WRIST_ANGLE_WAIT = 90;

    private static final double STARTING_CONFIG_PIVOT_ANGLE = 301.5;
    private static final double STARTING_CONFIG_HEIGHT = Units.inchesToMeters(0.0);
    private static final double STARTING_CONFIG_WRIST_ANGLE = 0;


    private static final double STOW_PIVOT_ANGLE = 180;
    private static final double STOW_HEIGHT = Units.inchesToMeters(0.0);
    private static final double STOW_WRIST_ANGLE = 0;

    private static final double TRANSFER_PIVOT_ANGLE = 5;
    private static final double TRANSFER_HEIGHT = Units.inchesToMeters(6.328);
    private static final double TRANSFER_WRIST_ANGLE = 90;


    private static final double BARGE_HEIGHT = Units.inchesToMeters(40);
    private static final double BARGE_PIVOT_ANGLE = 136.0;
    private static final double BARGE_WRIST_ANGLE = 0;

    private static final double ALT_BARGE_HEIGHT = Units.inchesToMeters(40);  //TODO FIX ME
    private static final double ALT_BARGE_PIVOT_ANGLE = 136;
    private static final double ALT_BARGE_WRIST_ANGLE = 0;
    
    private static final double FRONT_INTAKE_HEIGHT = 0;
    private static final double FRONT_INTAKE_PIVOT_ANGLE = 245;
    private static final double FRONT_INTAKE_WRIST_ANGLE = 90;


    private static final double BACK_INTAKE_HEIGHT = Units.inchesToMeters(4.5);
    private static final double BACK_INTAKE_PIVOT_ANGLE = 127.5;
    private static final double BACK_INTAKE_WRIST_ANGLE = 0;
    
    private static final double L2_ALGAE_HEIGHT= Units.inchesToMeters(7.0);
    private static final double L2_ALGAE_PIVOT_ANGLE = 270.0;
    private static final double L2_ALGAE_WRIST_ANGLE = 0;
    
    private static final double L3_ALGAE_HEIGHT= Units.inchesToMeters(24.0);
    private static final double L3_ALGAE_PIVOT_ANGLE = 270;    
    private static final double L3_ALGAE_WRIST_ANGLE = 0;

    private static final double PROCESSOR_HEIGHT = Units.inchesToMeters(0);
    private static final double PROCESSOR_PIVOT_ANGLE = 300;
    private static final double PROCESSOR_WRIST_ANGLE = 90;

    private static final double CLIMB_PIVOT_ANGLE = 0;
    private static final double CLIMB_HEIGHT = 4.5;
    private static final double CLIMB_WRIST_ANGLE = 0;

    private static final double ALGAE_TRANSFER_HEIGHT = Units.inchesToMeters(0.0);
    private static final double ALGAE_TRANSFER_PIVOT_ANGLE = 0.0;
    private static final double ALGAE_TRANSFER_WRIST_ANGLE = 0.0;



    // support delaying the elevator motion for a little bit
    // allows the pivot to start moving out of the way
    private static final double ELEVATOR_DELAY_HEIGHT = L4_HEIGHT - 0.1;
    private static final double ELEVATOR_DELAY_TIME = 0.1;
    private Timer m_elevatorTimer = new Timer();
    private Timer m_wristTimer = new Timer();
    private boolean m_wristSet = false;

    private static final HashMap<Position, Triplet<Double, Double, Double>> POSITIONS = new HashMap<Position, Triplet<Double, Double, Double>>() {
        {
            put(Position.L1, new Triplet<Double, Double,Double>(L1_HEIGHT, L1_PIVOT_ANGLE, L1_WRIST_ANGLE));
            put(Position.L2, new Triplet<Double, Double,Double>(L2_HEIGHT, L2_PIVOT_ANGLE, L2_WRIST_ANGLE));
            put(Position.L3, new Triplet<Double, Double,Double>(L3_HEIGHT, L3_PIVOT_ANGLE, L3_WRIST_ANGLE));
            put(Position.L4, new Triplet<Double, Double,Double>(L4_HEIGHT, L4_PIVOT_ANGLE, L4_WRIST_ANGLE));
            put(Position.L2_PREP, new Triplet<Double, Double,Double>(L2_HEIGHT_PREP, L2_PIVOT_ANGLE_PREP, L2_WRIST_ANGLE_PREP));
            put(Position.L3_PREP, new Triplet<Double, Double,Double>(L3_HEIGHT_PREP, L3_PIVOT_ANGLE_PREP, L3_WRIST_ANGLE_PREP));
            put(Position.L4_PREP, new Triplet<Double, Double,Double>(L4_HEIGHT_PREP, L4_PIVOT_ANGLE_PREP, L4_WRIST_ANGLE_PREP));

            put(Position.L1_ALT, new Triplet<Double, Double,Double>(L1_HEIGHT_ALT, L1_PIVOT_ANGLE_ALT, L1_WRIST_ANGLE_ALT));
            put(Position.L2_ALT, new Triplet<Double, Double,Double>(L2_HEIGHT_ALT, L2_PIVOT_ANGLE_ALT, L2_WRIST_ANGLE_ALT));
            put(Position.L3_ALT, new Triplet<Double, Double,Double>(L3_HEIGHT_ALT, L3_PIVOT_ANGLE_ALT, L3_WRIST_ANGLE_ALT));
            put(Position.L4_ALT, new Triplet<Double, Double,Double>(L4_HEIGHT_ALT, L4_PIVOT_ANGLE_ALT, L4_WRIST_ANGLE_ALT));
            put(Position.L2_PREP_ALT, new Triplet<Double, Double,Double>(L2_HEIGHT_PREP_ALT, L2_PIVOT_ANGLE_PREP_ALT, L2_WRIST_ANGLE_PREP_ALT));
            put(Position.L3_PREP_ALT, new Triplet<Double, Double,Double>(L3_HEIGHT_PREP_ALT, L3_PIVOT_ANGLE_PREP_ALT, L3_WRIST_ANGLE_PREP_ALT));
            put(Position.L4_PREP_ALT, new Triplet<Double, Double,Double>(L4_HEIGHT_PREP_ALT, L4_PIVOT_ANGLE_PREP_ALT, L4_WRIST_ANGLE_PREP_ALT));

            put(Position.BARGE, new Triplet<Double, Double,Double>(ALT_BARGE_HEIGHT, ALT_BARGE_PIVOT_ANGLE, ALT_BARGE_WRIST_ANGLE));
            put(Position.FRONT_INTAKE, new Triplet<Double, Double,Double>(FRONT_INTAKE_HEIGHT, FRONT_INTAKE_PIVOT_ANGLE, FRONT_INTAKE_WRIST_ANGLE));
            put(Position.BACK_INTAKE, new Triplet<Double, Double,Double>(BACK_INTAKE_HEIGHT, BACK_INTAKE_PIVOT_ANGLE, BACK_INTAKE_WRIST_ANGLE));
            put(Position.L2_ALGAE, new Triplet<Double, Double,Double>(L2_ALGAE_HEIGHT, L2_ALGAE_PIVOT_ANGLE, L2_ALGAE_WRIST_ANGLE));
            put(Position.L3_ALGAE, new Triplet<Double, Double,Double>(L3_ALGAE_HEIGHT, L3_ALGAE_PIVOT_ANGLE, L3_ALGAE_WRIST_ANGLE));
            put(Position.STOW, new Triplet<Double, Double,Double>(STOW_HEIGHT, STOW_PIVOT_ANGLE, STOW_WRIST_ANGLE));
            put(Position.PROCESSOR, new Triplet<Double, Double,Double>(PROCESSOR_HEIGHT, PROCESSOR_PIVOT_ANGLE, PROCESSOR_WRIST_ANGLE));
            put(Position.CLIMB, new Triplet<Double, Double,Double>(CLIMB_HEIGHT, CLIMB_PIVOT_ANGLE, CLIMB_WRIST_ANGLE));
            put(Position.TRANSFER, new Triplet<Double,Double,Double>(TRANSFER_HEIGHT, TRANSFER_PIVOT_ANGLE, TRANSFER_WRIST_ANGLE));
            put(Position.TRANSFER_WAIT, new Triplet<Double,Double,Double>(TRANSFER_HEIGHT_WAIT, TRANSFER_PIVOT_ANGLE_WAIT, TRANSFER_WRIST_ANGLE_WAIT));
            put(Position.ALGAE_TRANSFER, new Triplet<Double,Double,Double>(ALGAE_TRANSFER_HEIGHT, ALGAE_TRANSFER_PIVOT_ANGLE, ALGAE_TRANSFER_WRIST_ANGLE));
            put(Position.STARTING_CONFIG, new Triplet<Double,Double,Double>(STARTING_CONFIG_HEIGHT, STARTING_CONFIG_PIVOT_ANGLE, STARTING_CONFIG_WRIST_ANGLE));

        }
    };
    
    public MoveEndEffectorRedesign(Constants.Position position, Elevator elevator, EndEffectorPivot pivot, EndEffectorWrist wrist) {
       this(position, elevator, pivot, wrist, 2.0, ()->false);
    }

    public MoveEndEffectorRedesign(Constants.Position position, Elevator elevator, EndEffectorPivot pivot, EndEffectorWrist wrist, BooleanSupplier wantsAltMode) {
        this(position, elevator, pivot, wrist, 2.0, wantsAltMode);
     }

    public MoveEndEffectorRedesign(Constants.Position position, Elevator elevator, EndEffectorPivot pivot, EndEffectorWrist wrist, double timeout, BooleanSupplier isAltMode) {
        m_pivot = pivot;
        m_elevator = elevator;
        m_position = position;
        m_wrist = wrist;
        m_timeoutDelay = timeout;
        m_isAltMode = isAltMode;
        Triplet<Double, Double, Double> desiredPos;

        System.out.println("Alt mode supplier returns: " + m_isAltMode.getAsBoolean());
        System.out.println("Original position: " + m_position);
    



        if(m_isAltMode.getAsBoolean() == true){
            m_newPos = returnAltPosition(m_position);
        }else{
            m_newPos = m_position;
        }

        System.out.println("Using position: " + m_newPos);

        desiredPos = POSITIONS.get(m_newPos);

        m_desiredHeight = desiredPos.getValue0();
        m_desiredPivotAngle = Rotation2d.fromDegrees(desiredPos.getValue1());
        m_desiredWristAngle = Rotation2d.fromDegrees(desiredPos.getValue2());
        
        // Require the elevator and pivot, since we are waiting for them to reach goal
        addRequirements(elevator, pivot, wrist);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("starting MoveEERedesign to " + m_position);
        m_commandTimeout.restart();

        m_pivot.setAngle(m_desiredPivotAngle);

        m_elevator.setHeight(m_desiredHeight);

        // figure out whether to set the elevator immediately, or delay a bit
        m_wristSet = false;
        // m_commandTimeout.restart();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double pivotAngle = m_pivot.getAngle().getDegrees();
        if (! m_wristSet && pivotAngle > 30 && pivotAngle < 270) {
            m_wrist.setAngle(m_desiredWristAngle);
            m_wristSet = true;
        }

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

    public Position returnAltPosition(Position pos){
        if (pos == null) {
            System.err.println("Error: position is null!");
            return Position.STOW; 
        }
        
        switch(pos){
            case L1: 
                return Position.L1_ALT;
            case L2: 
                return Position.L2_ALT;
            case L3: 
                return Position.L3_ALT;
            case L4: 
                return Position.L4_ALT;
            case L2_PREP: 
                return Position.L2_PREP_ALT;
            case L3_PREP: 
                return Position.L3_PREP_ALT;
            case L4_PREP: 
                return Position.L4_PREP_ALT;
            default: 
                System.err.println("return alt position, bad argument:" + pos + "does not have a defined alt" );   
                return pos;  
                    
        }
        
    }
}
