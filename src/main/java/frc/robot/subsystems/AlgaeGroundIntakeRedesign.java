// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.crypto.spec.PBEKeySpec;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeGroundIntakeRedesign extends SubsystemBase {
  //IMPORTANT NOTE: All angles are relitive to stowed which is zero in the code.

    private static final double ANGLE_TOLERANCE_DEG = 8.0;

    private static final int CURRENT_LIMIT = 60;

    private static final double GEAR_RATIO = 5.0/1.0;

    // Constants for the pivot PID controller
    private static final double K_P = 4;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;

    private final SparkMax m_pivotMotor;

    // private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_pivotController;

    private static final Rotation2d STOWED_ANGLE = Rotation2d.fromDegrees(0.0);
    private static final Rotation2d TRANSFER_ANGLE = Rotation2d.fromDegrees(0.0);

    // This is RPM


    private double m_goalAngle; //Used for readout in elastic only

    //Trapisoidl:
    private static final double MAX_VEL_ROT_PER_SEC = 3;
    private static final double MAX_ACC_ROT_PER_SEC2 = 2;
    private static final double ROBOT_LOOP_PERIOD = 0.02;
    
    
        // Trapezoid Profile
        private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VEL_ROT_PER_SEC, MAX_ACC_ROT_PER_SEC2));
        private State m_currentState = new State();
    

    
        // Construct a new shooterPivot subsystem
        public AlgaeGroundIntakeRedesign() {
            m_pivotMotor = new SparkMax(Constants.ALGAE_GROUND_PIVOT_ID, MotorType.kBrushless);
    
            SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
            pivotMotorConfig.inverted(false);
            pivotMotorConfig.idleMode(IdleMode.kBrake);
            pivotMotorConfig.smartCurrentLimit(CURRENT_LIMIT);
            pivotMotorConfig.encoder.positionConversionFactor(GEAR_RATIO);
    
            pivotMotorConfig.closedLoop.p(K_P).i(K_I).d(K_D);
    
            pivotMotorConfig.closedLoop.outputRange(-1, 1);
            pivotMotorConfig.closedLoop.positionWrappingEnabled(false); // don't treat it as a circle
    
            m_pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_pivotMotor.getEncoder().setPosition(0);
    
            SparkMaxConfig rollerMotor = new SparkMaxConfig();
            rollerMotor.inverted(true);
            rollerMotor.idleMode(IdleMode.kBrake);
            rollerMotor.smartCurrentLimit(CURRENT_LIMIT);
        
            // controller for PID control
            m_pivotController = m_pivotMotor.getClosedLoopController();
        }
    
        @Override
        public void periodic() {
            SmartDashboard.putNumber("algaeGroundIntake/goalAngle", m_goalAngle);
            SmartDashboard.putNumber("algaeGroundIntake/angleOffFromGoal", m_goalAngle - getPivotAngle().getDegrees());
            SmartDashboard.putNumber("algaeGroundIntake/currentAngle", getPivotAngle().getDegrees());
            //Use triggers to intake/outtake
        }

    private void setAngleWithProfile(Rotation2d angle) {
        m_goalAngle = angle.getDegrees();

        State goalState = new State(angle.getRotations(), 0);

        // Trapezoid Profile
        m_currentState = m_profile.calculate(ROBOT_LOOP_PERIOD, m_currentState, goalState);

        m_pivotController.setReference(m_currentState.position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    // Get the current pivot angle
    public Rotation2d getPivotAngle() {
        return Rotation2d.fromRotations(m_pivotMotor.getEncoder().getPosition());
    }

    public void setPivotAngle(Rotation2d angle, double feedForward) {
        m_goalAngle = angle.getDegrees();
        m_pivotController.setReference(angle.getRotations(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward);
        // SmartDashboard.putNumber("pivot/goal", m_goal.getDegrees());
    }



    // state changes
    public void Stow() {
        setAngleWithProfile(STOWED_ANGLE);
    }

    public void Transfer(){
        setAngleWithProfile(TRANSFER_ANGLE);
    }
}
