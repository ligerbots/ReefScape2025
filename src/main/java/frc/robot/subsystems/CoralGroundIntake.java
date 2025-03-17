// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralGroundIntake extends SubsystemBase {
  public enum CoralGroundIntakeState {
    STOWED, DEPLOYED, SCORING
  }

  // NOTE: All constants were taken from the 2023 arm
  // Note: Current values for limits are refrenced with the shooter being flat
  // facing fowards as zero.
  // As of writing the above note we still may want to change the limits
  public static final double ANGLE_TOLERANCE_DEG = 1.0;

  private static final int CURRENT_LIMIT = 60;

  private static final double MIN_ANGLE = -90.0; // FIXME: Update with real values
  private static final double MAX_ANGLE = 90.0; // FIXME: Update with real values

  private static final double GEAR_RATIO = 0; // FIXME: Update with real values

  // Constants for the pivot PID controller
  private static final double K_P = 5.0;
  private static final double K_I = 0.0;
  private static final double K_D = 0.0;

  private final SparkMax m_pivot_motor;
  private final SparkFlex m_roller_motor;

  // private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_controller;

  private final double STOWED_ANGLE = 0.0; //FIXME: Find the angle
  private final double DEPLOYED_ANGLE = 0.0; //FIXME: Find the angle
  private final double SCORING_ANGLE = 0.0; //FIXME: Find the angle

  // Construct a new shooterPivot subsystem
  public CoralGroundIntake() {
    m_pivot_motor = new SparkMax(Constants.CORAL_GROUND_PIVOT_ID, MotorType.kBrushless);
    m_roller_motor = new SparkFlex(Constants.CORAL_GROUND_ROLLER_ID, MotorType.kBrushless);

    SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    pivotMotorConfig.inverted(true);
    pivotMotorConfig.idleMode(IdleMode.kBrake);
    pivotMotorConfig.smartCurrentLimit(CURRENT_LIMIT);
    pivotMotorConfig.encoder.positionConversionFactor(GEAR_RATIO / 360.0); // convert rotations to degrees

    pivotMotorConfig.closedLoop.p(K_P).i(K_I).d(K_D);

    pivotMotorConfig.closedLoop.outputRange(-1, 1);
    pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    pivotMotorConfig.closedLoop.positionWrappingEnabled(false); // don't treat it as a circle

    m_pivot_motor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rollerMotor = new SparkMaxConfig();
    rollerMotor.inverted(true);
    rollerMotor.idleMode(IdleMode.kBrake);
    rollerMotor.smartCurrentLimit(CURRENT_LIMIT);

    rollerMotor.closedLoop.p(K_P).i(K_I).d(K_D);

    rollerMotor.closedLoop.outputRange(-1, 1);
    rollerMotor.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    rollerMotor.closedLoop.positionWrappingEnabled(false); // don't treat it as a circle
    m_roller_motor.configure(rollerMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // motor encoder - set calibration and offset to match absolute encoder
    // m_encoder = m_motor.getEncoder();

    // controller for PID control
    m_controller = m_pivot_motor.getClosedLoopController();
  }

  @Override
  public void periodic() {

    // double oldGoalClipped = m_goalClipped.getDegrees();

    // boolean goalChanged = Math.abs(m_goalClipped.getDegrees() - oldGoalClipped) >
    // 5.0;
  }

  // get the current pivot angle
  public Rotation2d getPivotAngle() {
    return Rotation2d.fromRotations(m_pivot_motor.getEncoder().getPosition());
  }

  // set shooterPivot angle
  public void setPivotAngle(Rotation2d angle) {
    m_controller.setReference(angle.getRotations(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    // SmartDashboard.putNumber("pivot/goal", m_goal.getDegrees());
  }

  // needs to be public so that commands can get the restricted angle
  public Rotation2d limitPivotAngle(Rotation2d angle) {
    double angleClamped = MathUtil.clamp(angle.getDegrees(), MIN_ANGLE, MAX_ANGLE);
    return Rotation2d.fromDegrees(angleClamped);
  }
}