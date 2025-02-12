// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class EndEffectorPivot extends SubsystemBase {
    
    // When the elevator is low, the angle range needs to be restricted
    // Remember that the midpoint is roughly 180 (not 0).
    private static final double MIN_ANGLE_LOW_DEG = 139.0;
    private static final double MAX_ANGLE_LOW_DEG = 298.0;

    private static final double MIN_ANGLE_HIGH_DEG = 110.0;
    private static final double MAX_ANGLE_HIGH_DEG = 350.0;

    // NOTE: All constants were taken from the 2023 arm 
    // Note: Current values for limits are refrenced with the shooter being flat
    // facing fowards as zero.
    // As of writing the above note we still may want to change the limits
    public static final double ANGLE_TOLERANCE_DEG = 1.0;

    private static final int CURRENT_LIMIT = 30;
    
    // 25:1 planetary plus 42:18 sprockets
    private static final double GEAR_RATIO = 25.0 * (42.0 / 18.0);
      
    // Constants to limit the shooterPivot rotation speed
    // max vel: 1 rotation = 10 seconds  and then gear_ratio
    private static final double MAX_VEL_ROT_PER_SEC = 0.2 * GEAR_RATIO;
    private static final double MAX_ACC_ROT_PER_SEC2 = 1.0 * GEAR_RATIO;
    // units??? rotations?
    private static final double ALLOWED_ERROR = ANGLE_TOLERANCE_DEG/360.0 * GEAR_RATIO;

    // lower max vel and acceleration for when the elevator is high
    private static final double MAX_VEL_SLOW_ROT_PER_SEC = 0.5 * MAX_VEL_ROT_PER_SEC;
    private static final double MAX_ACC_SLOW_ROT_PER_SEC2 = 0.5 * MAX_ACC_ROT_PER_SEC2;

    // Zero point of the absolute encoder
    private static final double ABS_ENCODER_ZERO_OFFSET = (135.2+180)/360.0; 

    // Constants for the pivot PID controller
    private static final double K_P = 2.0;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;
    private static final double K_FF = 0.0;

    // feedforward terms to balance gravity
    // TODO: find good values
    private static final Rotation2d BALANCE_ANGLE = Rotation2d.fromDegrees(160);
    // FF scale. Units are Volts (probably negative)
    private static final double K_GRAVITY = 0;

    // aliases names for the slots, so it is easier to remember
    private static final ClosedLoopSlot SLOT_FAST = ClosedLoopSlot.kSlot0;
    private static final ClosedLoopSlot SLOT_SLOW = ClosedLoopSlot.kSlot1;

    private final SparkMax m_motor;
    private final SparkAbsoluteEncoder m_absoluteEncoder;
    private final SparkAbsoluteEncoderSim m_absoluteEncoderSim;
    private final SparkClosedLoopController m_controller;

    // Used for checking if on goal
    // m_goal is the commanded goal
    // m_goalClipped is the goal taking into account any restrictions
    private Rotation2d m_goal = Rotation2d.fromDegrees(0);
    private Rotation2d m_goalClipped = Rotation2d.fromDegrees(0);

    // supplier to get the Elevator height
    // needed to check on range limits and motion speed
    private final DoubleSupplier m_elevatorHeight;

    // Construct a new shooterPivot subsystem
    public EndEffectorPivot(DoubleSupplier elevatorHeight) {
        m_elevatorHeight = elevatorHeight;
        
        m_motor = new SparkMax(Constants.END_EFFECTOR_PIVOT_CAN_ID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(CURRENT_LIMIT);

        AbsoluteEncoderConfig absEncConfig = new AbsoluteEncoderConfig();
        absEncConfig.velocityConversionFactor(1/60.0);   // convert rpm to rps
        absEncConfig.zeroOffset(ABS_ENCODER_ZERO_OFFSET);
        absEncConfig.inverted(false);
        config.apply(absEncConfig);

        // General parameters for the cloed loop/MAXMotion control
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(false); // don't treat it as a circle
        
        // Set up the "fast" slot for typical control (when the elevator is not high)
        config.closedLoop
                .pidf(K_P, K_I, K_D, K_FF, SLOT_FAST)
                .outputRange(-1, 1, SLOT_FAST);
        // MAXMotion parameters
        config.closedLoop.maxMotion
                .maxVelocity(MAX_VEL_ROT_PER_SEC, SLOT_FAST)
                .maxAcceleration(MAX_ACC_ROT_PER_SEC2, SLOT_FAST)
                .allowedClosedLoopError(ALLOWED_ERROR, SLOT_FAST)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, SLOT_FAST);

        // Set up the "slow" slot; used when the elevator is high
        config.closedLoop
                .pidf(K_P, K_I, K_D, K_FF, SLOT_SLOW)
                .outputRange(-1, 1, SLOT_SLOW);
        config.closedLoop.maxMotion
                .maxVelocity(MAX_VEL_SLOW_ROT_PER_SEC, SLOT_SLOW)
                .maxAcceleration(MAX_ACC_SLOW_ROT_PER_SEC2, SLOT_SLOW)
                .allowedClosedLoopError(ALLOWED_ERROR, SLOT_SLOW)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, SLOT_SLOW);
                        
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_absoluteEncoder = m_motor.getAbsoluteEncoder();
        m_absoluteEncoderSim = new SparkAbsoluteEncoderSim(m_motor);
        m_absoluteEncoderSim.setZeroOffset(ABS_ENCODER_ZERO_OFFSET);

        // controller for MAX Motion
        m_controller = m_motor.getClosedLoopController();

        // set the goal to the current sensor value; should prevent sudden motion when enabling
        resetGoal();

        SmartDashboard.putBoolean("pivot/coastMode", false);
        setCoastMode();

        SmartDashboard.putNumber("pivot/testAngle", getAngle().getDegrees());
    }

    @Override
    public void periodic() {
        double elevHeight = m_elevatorHeight.getAsDouble();
        m_goalClipped = limitPivotAngle(m_goal, elevHeight);

        // Pick the slow to use based on Elevator height
        m_controller.setReference(m_goalClipped.getRotations(), ControlType.kMAXMotionPositionControl, 
                    elevHeight >= Elevator.SLOW_PIVOT_MIN_HEIGHT ? SLOT_SLOW : SLOT_FAST,
                    feedforward());

        // Display current values on the SmartDashboard
        // These also get logged to the log file on the Rio and aid in replaying a match
        SmartDashboard.putNumber("pivot/goalClipped", m_goalClipped.getDegrees());
        SmartDashboard.putNumber("pivot/absoluteEncoder", getAngle().getDegrees());
        SmartDashboard.putNumber("pivot/outputCurrent", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("pivot/busVoltage", m_motor.getBusVoltage());
        SmartDashboard.putBoolean("pivot/onGoal", angleWithinTolerance());
        SmartDashboard.putNumber("pivot/appliedOutput", m_motor.getAppliedOutput());
    }

    // get the current pivot angle
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_absoluteEncoder.getPosition());
    }

    // only for testing!!
    public void run(double speed) {
        m_motor.set(speed);
    }

    // set shooterPivot angle
    public void setAngle(Rotation2d angle) {
        m_goal = angle;
        SmartDashboard.putNumber("pivot/goal", m_goal.getDegrees());
    }
    
    // get the angle from the absolute encoder
    // public double getAbsEncoderAngleRadians() {
    //     return TWO_PI * m_absoluteEncoder.getDistance();
    // }

    // // update the motor encoder offset to match the absolute encoder
    // public void updateMotorEncoderOffset() {
    //     m_encoder.setPosition(m_absoluteEncoder.getDistance());
    // }

    public boolean isOutsideLowRange() {
        double angle = getAngle().getDegrees();
        return angle <= MIN_ANGLE_LOW_DEG || angle >= MAX_ANGLE_LOW_DEG;
    }

    // needs to be public so that commands can get the restricted angle
    public Rotation2d limitPivotAngle(Rotation2d angle, double elevHeight) {
        double angleClamped;
        if (elevHeight <= Elevator.NARROW_PIVOT_MAX_HEIGHT)
            angleClamped = MathUtil.clamp(angle.getDegrees(), MIN_ANGLE_LOW_DEG, MAX_ANGLE_LOW_DEG);
        else
            angleClamped = MathUtil.clamp(angle.getDegrees(), MIN_ANGLE_HIGH_DEG, MAX_ANGLE_HIGH_DEG);

        return Rotation2d.fromDegrees(angleClamped);
    }

    public boolean angleWithinTolerance() {
        //TODO does MAXMotion provide this?
        return Math.abs(m_goalClipped.minus(getAngle()).getDegrees()) < ANGLE_TOLERANCE_DEG;
    }

    private double feedforward() {
        Rotation2d angle = getAngle().minus(BALANCE_ANGLE);
        return K_GRAVITY * angle.getSin();
    }

    // public void adjustAngle(boolean goUp) {
    //     double adjust = (goUp ? 1 : -1) * ADJUSTMENT_STEP;
    //     m_angleAdjustment += adjust;
    //     setAngle(m_goalRadians + adjust, false);
    // }

    public void resetGoal() {
        setAngle(getAngle());
    }

    public void setCoastMode() {
        boolean coastMode = SmartDashboard.getBoolean("shooterPivot/coastMode", false);
        if (coastMode) {
            m_motor.stopMotor();
            m_motor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        } else
            m_motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}