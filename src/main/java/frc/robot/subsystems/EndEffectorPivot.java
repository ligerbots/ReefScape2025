// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
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
    
    public static final double MIN_ANGLE_DEG = Math.toRadians(0.0);
    public static final double MAX_ANGLE_DEG = Math.toRadians(60.0);
    // NOTE: All constants were taken from the 2023 arm 
    // Note: Current values for limits are refrenced with the shooter being flat
    // facing fowards as zero.
    // As of writing the above note we still may want to change the limits
    public static final double ANGLE_TOLERANCE_DEG = 1.0;

    private static final int CURRENT_LIMIT = 30;

    // position constants for commands
    // private static final double ADJUSTMENT_STEP = Math.toRadians(1.0);
    
    // 25:1 planetary plus 42:18 sprockets
    private static final double GEAR_RATIO = 25.0 * (42.0 / 18.0);
      
    // Constants to limit the shooterPivot rotation speed
    // max vel: 1 rotation = 10 seconds  and then gear_ratio
    // "60" because they want RPM
    private static final double MAX_VEL_ROT_PER_SEC = 0.1 * GEAR_RATIO;
    private static final double MAX_ACC_ROT_PER_SEC2 = 0.2 * GEAR_RATIO;
    // units??? rotations?
    private static final double ALLOWED_ERROR = 2.0/360.0 * GEAR_RATIO;

    private static final double POSITION_OFFSET = 238.8/360.0; 
    // private static final double OFFSET_RADIAN = POSITION_OFFSET * 2 * Math.PI;

    // Constants for the shooterPivot PID controller
    private static final double K_P = 1.0;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;
    private static final double K_FF = 0.0;

    // Used in conversion factor
    // private static final double RADIANS_PER_MOTOR_ROTATION = 2 * Math.PI * GEAR_RATIO;

    // private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(0);  
    private final SparkMax m_motor;
    // private final RelativeEncoder m_encoder;
    private final SparkAbsoluteEncoder m_absoluteEncoder;
    private final SparkAbsoluteEncoderSim m_absoluteEncoderSim;
    private final SparkClosedLoopController m_controller;

    // Used for checking if on goal
    private Rotation2d m_goal = Rotation2d.fromDegrees(0);

    // adjustment offset. Starts at 0, but retained throughout a match
    // private double m_angleAdjustment = Math.toRadians(0.0);

    // Construct a new shooterPivot subsystem
    public EndEffectorPivot() {
        m_motor = new SparkMax(Constants.END_EFFECTOR_PIVOT_CAN_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit(CURRENT_LIMIT);
    
        AbsoluteEncoderConfig absConfig = new AbsoluteEncoderConfig();
        absConfig.velocityConversionFactor(1/60.0);   // convert rpm to rps
        absConfig.zeroOffset(POSITION_OFFSET);
        config.apply(absConfig);
        
        // set up the PID for MAX Motion
        config.closedLoop.pidf(K_P, K_I, K_D, K_FF);

        config.closedLoop.outputRange(-1, 1);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.closedLoop.positionWrappingEnabled(true);
        config.closedLoop.positionWrappingInputRange(0,1.0);

        // Set Smart Motion and Smart Velocity parameters.
        config.closedLoop.maxMotion
                .maxVelocity(MAX_VEL_ROT_PER_SEC)
                .maxAcceleration(MAX_ACC_ROT_PER_SEC2)
                .allowedClosedLoopError(ALLOWED_ERROR);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // motor encoder - set calibration and offset to match absolute encoder
        // m_encoder = m_motor.getEncoder();

        m_absoluteEncoder = m_motor.getAbsoluteEncoder();
        m_absoluteEncoderSim = new SparkAbsoluteEncoderSim(m_motor);
        m_absoluteEncoderSim.setZeroOffset(POSITION_OFFSET);

        // controller for MAX Motion
        m_controller = m_motor.getClosedLoopController();

        // updateMotorEncoderOffset();
        // resetGoal();

        SmartDashboard.putBoolean("pivot/coastMode", false);
        setCoastMode();

        SmartDashboard.putNumber("pivot/testAngle", 0);
    }

    @Override
    public void periodic() {
        // Display current values on the SmartDashboard
        // This also gets logged to the log file on the Rio and aids in replaying a match
        SmartDashboard.putNumber("pivot/absoluteEncoder", getPosition().getDegrees());
        SmartDashboard.putNumber("pivot/current", m_motor.getOutputCurrent());
        SmartDashboard.putBoolean("pivot/onGoal", angleWithinTolerance());

        setCoastMode();
    }

    // get the current pivot angle
    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(m_absoluteEncoder.getPosition());
    }

    // set shooterPivot angle
    public void setAngle(Rotation2d angle) {
        m_goal = limitPivotAngle(angle);
        m_controller.setReference(m_goal.getRotations(), SparkBase.ControlType.kMAXMotionPositionControl);
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

    // needs to be public so that commands can get the restricted angle
    public static Rotation2d limitPivotAngle(Rotation2d angle) {
        return Rotation2d.fromDegrees(MathUtil.clamp(angle.getDegrees(), MIN_ANGLE_DEG, MAX_ANGLE_DEG));
    }

    public boolean angleWithinTolerance() {
        //TODO does MAXMotion provide this?
        return Math.abs(m_goal.minus(getPosition()).getDegrees()) < ANGLE_TOLERANCE_DEG;
    }

    // public void adjustAngle(boolean goUp) {
    //     double adjust = (goUp ? 1 : -1) * ADJUSTMENT_STEP;
    //     m_angleAdjustment += adjust;
    //     setAngle(m_goalRadians + adjust, false);
    // }

    public void resetGoal() {
        setAngle(getPosition());
    }

    public void setCoastMode() {
        boolean coastMode = SmartDashboard.getBoolean("shooterPivot/coastMode", false);
        if (coastMode) {
            m_motor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            m_motor.stopMotor();
        } else
        m_motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}