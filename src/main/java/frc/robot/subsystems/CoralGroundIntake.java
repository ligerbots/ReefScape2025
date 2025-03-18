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
import frc.robot.subsystems.ValueThreshold.Direction;

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
  private final SparkClosedLoopController m_pivotController;
  private final SparkClosedLoopController m_rollerController;

  private final double STOWED_ANGLE = 0.0; //FIXME: Find the angle
  private final double DEPLOYED_ANGLE = 0.0; //FIXME: Find the angle
  private final double SCORING_ANGLE = 0.0; //FIXME: Find the angle

  private final double ROLLER_INTAKE_SPEED_PRECENT = 0.0; //FIXME: Find the speed
  private final double ROLLER_OUTTAKE_SPEED_PRECENT = 0.0; //FIXME: Find the speed

  private static final double STALL_VELOCITY_LIMIT = 2000; //TODO: Find a good value
  private final ValueThreshold m_speedThres = new ValueThreshold(Direction.FALLING, STALL_VELOCITY_LIMIT);

  public CoralGroundIntakeState m_state = CoralGroundIntakeState.STOWED;

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
    pivotMotorConfig.closedLoop.positionWrappingEnabled(false); // don't treat it as a circle

    m_pivot_motor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rollerMotor = new SparkMaxConfig();
    rollerMotor.inverted(true);
    rollerMotor.idleMode(IdleMode.kBrake);
    rollerMotor.smartCurrentLimit(CURRENT_LIMIT);

    rollerMotor.closedLoop.p(K_P).i(K_I).d(K_D);

    rollerMotor.closedLoop.outputRange(-1, 1);
    m_roller_motor.configure(rollerMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // motor encoder - set calibration and offset to match absolute encoder
    // m_encoder = m_motor.getEncoder();

    // controller for PID control
    m_pivotController = m_pivot_motor.getClosedLoopController();
    m_rollerController = m_roller_motor.getClosedLoopController();
  }

  @Override
  public void periodic() {

    // double oldGoalClipped = m_goalClipped.getDegrees();

    // boolean goalChanged = Math.abs(m_goalClipped.getDegrees() - oldGoalClipped) >
    // 5.0;

    switch (m_state) {
      case STOWED:
        setPivotAngle(Rotation2d.fromDegrees(STOWED_ANGLE));
        setRollerSpeedPrecent(0); //Note: This may want to be a activly intaking number so the coral does not fall out.
        break;
      case DEPLOYED:
        boolean stalled = m_speedThres.compute(Math.abs(getRollerSpeed().getRadians()));
        if (stalled) {
          setRollerSpeedPrecent(0);
          m_state = CoralGroundIntakeState.STOWED;
        } else {
          setRollerSpeedPrecent(ROLLER_INTAKE_SPEED_PRECENT);
          setPivotAngle(Rotation2d.fromDegrees(DEPLOYED_ANGLE));
        }
        break;
      case SCORING:
        setPivotAngle(Rotation2d.fromDegrees(SCORING_ANGLE));
        if (ANGLE_TOLERANCE_DEG > Math.abs(getPivotAngle().getDegrees() - SCORING_ANGLE)) {
          setRollerSpeedPrecent(ROLLER_OUTTAKE_SPEED_PRECENT);
        } else {
          setRollerSpeedPrecent(0);
        }

        break;
    }
  }

  public Rotation2d getRollerSpeed() {
    return Rotation2d.fromRotations(m_roller_motor.getEncoder().getVelocity());
  }

  // get the current pivot angle
  public Rotation2d getPivotAngle() {
    return Rotation2d.fromRotations(m_pivot_motor.getEncoder().getPosition());
  }

  public void setPivotAngle(Rotation2d angle) {
    m_pivotController.setReference(angle.getRotations(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    // SmartDashboard.putNumber("pivot/goal", m_goal.getDegrees());
  }

  // set the speed of the roller in RPM
  public void setRollerSpeedPrecent(double speed) {
    m_roller_motor.set(speed);
  }

  // needs to be public so that commands can get the restricted angle
  public Rotation2d limitPivotAngle(Rotation2d angle) {
    double angleClamped = MathUtil.clamp(angle.getDegrees(), MIN_ANGLE, MAX_ANGLE);
    return Rotation2d.fromDegrees(angleClamped);
  }
}