// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.DoubleSupplier;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class EndEffectorPivot extends SubsystemBase {
    
    private static final double MIN_ANGLE_LOW_DEG = 140.0;
    private static final double MAX_ANGLE_LOW_DEG = 295.0;

    private static final double MIN_ANGLE_HIGH_DEG = 140.0;
    private static final double MAX_ANGLE_HIGH_DEG = 295.0;
    private static final double ELEVATOR_HEIGHT_LOW_RANGE = Units.inchesToMeters(5.0);

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
    private static final double MAX_VEL_ROT_PER_SEC = 0.1 * GEAR_RATIO;
    private static final double MAX_ACC_ROT_PER_SEC2 = 0.2 * GEAR_RATIO;
    // units??? rotations?
    private static final double ALLOWED_ERROR = 2.0/360.0 * GEAR_RATIO;

    // Zero point of the absolute encoder
    private static final double ABS_ENCODER_ZERO_OFFSET = (135.2+180)/360.0; 

    // Constants for the pivot PID controller
    private static final double K_P = 1.0;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;
    private static final double K_FF = 0.0;

    private final SparkMax m_motor;
    // private final RelativeEncoder m_encoder;
    private final SparkAbsoluteEncoder m_absoluteEncoder;
    private final SparkAbsoluteEncoderSim m_absoluteEncoderSim;
    private final SparkClosedLoopController m_controller;

    // Used for checking if on goal
    private Rotation2d m_goal = Rotation2d.fromDegrees(0);

    // adjustment offset. Starts at 0, but retained throughout a match
    // private double m_angleAdjustment = Math.toRadians(0.0);

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
        absEncConfig.setSparkMaxDataPortConfig();
        config.apply(absEncConfig);
        
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
        m_absoluteEncoderSim.setZeroOffset(ABS_ENCODER_ZERO_OFFSET);

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
        SmartDashboard.putNumber("pivot/absoluteEncoder", getAngle().getDegrees());
        SmartDashboard.putNumber("pivot/outputCurrent", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("pivot/busVoltage", m_motor.getBusVoltage());
        SmartDashboard.putBoolean("pivot/onGoal", angleWithinTolerance());
        // SmartDashboard.putNumber("pivot/rawAbsEncoder", m_absoluteEncoder.getPosition());

        setCoastMode();
    }

    // get the current pivot angle
    public Rotation2d getAngle() {
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
    public Rotation2d limitPivotAngle(Rotation2d angle) {
        double angleClamped;
        if (m_elevatorHeight.getAsDouble() <= ELEVATOR_HEIGHT_LOW_RANGE)
            angleClamped = MathUtil.clamp(angle.getDegrees(), MIN_ANGLE_LOW_DEG, MAX_ANGLE_LOW_DEG);
        else
            angleClamped = MathUtil.clamp(angle.getDegrees(), MIN_ANGLE_HIGH_DEG, MAX_ANGLE_HIGH_DEG);

        return Rotation2d.fromDegrees(angleClamped);
    }

    public boolean angleWithinTolerance() {
        //TODO does MAXMotion provide this?
        return Math.abs(m_goal.minus(getAngle()).getDegrees()) < ANGLE_TOLERANCE_DEG;
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
            m_motor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            m_motor.stopMotor();
        } else
        m_motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}