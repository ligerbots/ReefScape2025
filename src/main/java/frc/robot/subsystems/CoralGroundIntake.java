// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ValueThreshold.Direction;

public class CoralGroundIntake extends SubsystemBase {
    public enum CoralGroundIntakeState {
        STOW, DEPLOY, SCORE
    }

    //IMPORTANT NOTE: All angles are relitive to deployed which is zero in the code.

    // NOTE: All constants were taken from the 2023 arm
    // Note: Current values for limits are refrenced with the shooter being flat
    // facing fowards as zero.
    // As of writing the above note we still may want to change the limits
    public static final double ANGLE_TOLERANCE_DEG = 3.0;

    private static final int CURRENT_LIMIT = 60;
 
    private static final double MIN_ANGLE = 0; // FIXME: Update with real values
    private static final double MAX_ANGLE = 90.0; // FIXME: Update with real values

    private static final double GEAR_RATIO = 16.0/42.0 * 1.0/4.0; 

    // Constants for the pivot PID controller
    private static final double K_P = 2.5;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;

    private final SparkMax m_pivotMotor;
    private final SparkFlex m_rollerMotor;

    // private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_pivotController;

    private final double STOWED_ANGLE = 120.0; // FIXME: Find the angle
    private final double SCORING_ANGLE = 100.0; // FIXME: Find the angle
    private final double DEPLOYED_ANGLE = 2.0; // FIXME: Find the angle

    private final double ROLLER_INTAKE_SPEED = 0.4; // FIXME: Find the speed
    private final double ROLLER_OUTTAKE_SPEED = -0.2; // FIXME: Find the speed
    private final double ROLLER_HOLD_SPEED = 0.2; // FIXME: Find the speed

    // This is RPM
    private static final double STALL_VELOCITY_LIMIT = 10; // TODO: Find a good value

    public CoralGroundIntakeState m_state = CoralGroundIntakeState.DEPLOY;

    private double m_goalAngle; //Used for readout in elastic only

    private Timer m_stallTime = new Timer();

    //Trapisoidl:
    private static final double MAX_VEL_ROT_PER_SEC = 1.5;
    private static final double MAX_ACC_ROT_PER_SEC2 = 3.0/2;
    private static final double ROBOT_LOOP_PERIOD = 0.02;

    // Trapezoid Profile
    private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VEL_ROT_PER_SEC, MAX_ACC_ROT_PER_SEC2));
    private State m_currentState = new State();

    // Used for detecting if we have coral based on speed. Size should be: 1 / robot loop period(0.02 seconds) * 2 * Duration before we decide we have coral
   private final MedianFilter m_coralHoldFilter = new MedianFilter(30);


    // Construct a new shooterPivot subsystem
    public CoralGroundIntake() {
        m_pivotMotor = new SparkMax(Constants.CORAL_GROUND_PIVOT_ID, MotorType.kBrushless);
        m_rollerMotor = new SparkFlex(Constants.CORAL_GROUND_ROLLER_ID, MotorType.kBrushless);

        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        pivotMotorConfig.inverted(true);
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

        m_rollerMotor.configure(rollerMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // motor encoder - set calibration and offset to match absolute encoder
        // m_encoder = m_motor.getEncoder();

        // controller for PID control
        m_pivotController = m_pivotMotor.getClosedLoopController();
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("coralGroundIntake/angleOffFromGoal", m_goalAngle - getPivotAngle().getDegrees());
        boolean stalled = m_coralHoldFilter.calculate(Math.abs(getRollerSpeed().getRotations())) < STALL_VELOCITY_LIMIT;
        SmartDashboard.putBoolean("coralGroundIntake/isStalled", stalled);

        switch (m_state) {
            case STOW:
                setAngleWithProfile(Rotation2d.fromDegrees(STOWED_ANGLE));
                if (stalled) {
                    setRollerSpeedPercent(ROLLER_HOLD_SPEED); 
                } else {
                    setRollerSpeedPercent(0);
                }
                break;
            case DEPLOY:
                setRollerSpeedPercent(ROLLER_INTAKE_SPEED);
                setAngleWithProfile(Rotation2d.fromDegrees(DEPLOYED_ANGLE));

                if ((ANGLE_TOLERANCE_DEG > Math.abs(getPivotAngle().getDegrees() - ROLLER_INTAKE_SPEED)) && stalled) {
                    stow();
                    System.out.println("Stalled, intaking");
                }
                break;
            case SCORE:
                // TODO: Detect if a coral is scored, then automatically switch to stow
                setAngleWithProfile(Rotation2d.fromDegrees(SCORING_ANGLE));
                if (ANGLE_TOLERANCE_DEG > Math.abs(getPivotAngle().getDegrees() - SCORING_ANGLE)) {
                    setRollerSpeedPercent(ROLLER_OUTTAKE_SPEED);
                } else {
                    setRollerSpeedPercent(ROLLER_HOLD_SPEED);
                }
                break;
        }
    }

    private void setAngleWithProfile(Rotation2d angle) {

        State goalState = new State(angle.getRotations(), 0);

        // Trapezoid Profile
        m_currentState = m_profile.calculate(ROBOT_LOOP_PERIOD, m_currentState, goalState);

        m_pivotController.setReference(m_currentState.position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public Rotation2d getRollerSpeed() {
        return Rotation2d.fromRotations(m_rollerMotor.getEncoder().getVelocity());
    }

    // get the current pivot angle
    public Rotation2d getPivotAngle() {
        return Rotation2d.fromRotations(m_pivotMotor.getEncoder().getPosition());
    }

    public void setPivotAngle(Rotation2d angle, double feedForward) {
        m_goalAngle = angle.getDegrees();
        m_pivotController.setReference(angle.getRotations(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward); //TODO: Add feedfowards as last param
        // SmartDashboard.putNumber("pivot/goal", m_goal.getDegrees());
    }

    // set the speed of the roller in RPM
    public void setRollerSpeedPercent(double speed) {
        m_rollerMotor.set(speed);
    }

    // needs to be public so that commands can get the restricted angle
    public Rotation2d limitPivotAngle(Rotation2d angle) {
        double angleClamped = MathUtil.clamp(angle.getDegrees(), MIN_ANGLE, MAX_ANGLE);
        return Rotation2d.fromDegrees(angleClamped);
    }

    // state changes
    public void stow() {
        m_state = CoralGroundIntakeState.STOW;
    }

    public void deploy() {
        m_state = CoralGroundIntakeState.DEPLOY;
    }

    public void score() {
        m_state = CoralGroundIntakeState.SCORE;
    }
}