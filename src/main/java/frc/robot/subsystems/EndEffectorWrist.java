// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffectorWrist extends SubsystemBase {
    public static final double ANGLE_TOLERANCE_DEG = 1.0;

    private static final int CURRENT_LIMIT = 40;
    
    private static final double GEAR_RATIO = 1.0 / 90.0;

    // Constants to limit the wrist rotation speed
    private static final double MAX_VEL_ROT_PER_SEC = 1.5;
    private static final double MAX_ACC_ROT_PER_SEC2 = 3.0;
    private static final double ROBOT_LOOP_PERIOD = 0.02;

    // Zero point of the absolute encoder
    private static final double ABS_ENCODER_ZERO_OFFSET = -15.86 / 360.0;

    // Constants for the pivot PID controller
    private static final double K_P = 10.0;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;

    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_controller;

    private final CANcoder m_cancoder;

    // Used for checking if on goal
    private Rotation2d m_goal = Rotation2d.fromDegrees(0);
    private Rotation2d m_goalClipped = Rotation2d.fromDegrees(0);

    // Trapezoid Profile
    private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VEL_ROT_PER_SEC, MAX_ACC_ROT_PER_SEC2));
    private State m_currentState = new State();

    private final DoubleSupplier m_elevatorHeight;

    // Construct a new shooterPivot subsystem
    public EndEffectorWrist(DoubleSupplier elevatorHeight) {
        m_elevatorHeight = elevatorHeight;

        m_cancoder = new CANcoder(Constants.WRIST_CANCODER_CAN_ID, "rio");
        m_motor = new SparkMax(Constants.END_EFFECTOR_WRIST_CAN_ID, MotorType.kBrushless);

        // Configure the CANcoder
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1);
        cancoderConfig.MagnetSensor.withMagnetOffset(ABS_ENCODER_ZERO_OFFSET);
        cancoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        m_cancoder.getConfigurator().apply(cancoderConfig);

        // Configure the SparkMax
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(CURRENT_LIMIT);
        config.encoder.positionConversionFactor(GEAR_RATIO);

        // set up the PID for MAX Motion
        config.closedLoop.p(K_P).i(K_I).d(K_D);

        config.closedLoop.outputRange(-1, 1);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.positionWrappingEnabled(false);  // don't treat it as a circle
        // config.closedLoop.positionWrappingInputRange(0,1.0);
                        
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_motor.getEncoder();
        m_encoder.setPosition(m_cancoder.getAbsolutePosition().getValueAsDouble() / 2.0);
        
        // controller for PID control
        m_controller = m_motor.getClosedLoopController();

        // updateMotorEncoderOffset();
        resetGoal();

        SmartDashboard.putBoolean("wrist/coastMode", false);
        setCoastMode();

        SmartDashboard.putNumber("wrist/testAngle", 0);
    }

    @Override
    public void periodic() {
        double elevHeight = m_elevatorHeight.getAsDouble();

        // double oldGoalClipped = m_goalClipped.getDegrees();
        m_goalClipped = limitWristAngle(m_goal, elevHeight);
        // TODO: should find shortest turn to get to the desired angle, like with swerve modules

        // boolean goalChanged = Math.abs(m_goalClipped.getDegrees() - oldGoalClipped) > 5.0;

        State goalState = new State(m_goalClipped.getRotations(), 0);

        // Trapezoid Profile
        m_currentState = m_profile.calculate(ROBOT_LOOP_PERIOD, m_currentState, goalState);

        double angleDeg = getAngle().getDegrees();

        m_controller.setReference(m_currentState.position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);

        // Display current values on the SmartDashboard
        // This also gets logged to the log file on the Rio and aids in replaying a match
        SmartDashboard.putNumber("wrist/statePosition", m_currentState.position * 360.0);
        SmartDashboard.putNumber("wrist/goalClipped", m_goalClipped.getDegrees());
        SmartDashboard.putNumber("wrist/motorEncoder", angleDeg);
        SmartDashboard.putNumber("wrist/absEncoder", getAbsEncoderAngle().getDegrees());

        // encoder values without wrapping (but Rotation2d does not actually wrap!)
        // keep for debugging for now
        SmartDashboard.putNumber("wrist/motorEncoderNoWrap", m_encoder.getPosition() * 360.0);
        SmartDashboard.putNumber("wrist/absEncoderNoWrap", m_cancoder.getPosition().getValueAsDouble() * 360.0);

        SmartDashboard.putNumber("wrist/outputCurrent", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("wrist/busVoltage", m_motor.getBusVoltage());
        SmartDashboard.putBoolean("wrist/onGoal", angleWithinTolerance());
        SmartDashboard.putNumber("wrist/appliedOutput", m_motor.getAppliedOutput());
        SmartDashboard.putNumber("wrist/velocity", getVelocity().getDegrees());
    }

    // get the current wrist angle
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_encoder.getPosition());
    }

    public Rotation2d getAbsEncoderAngle(){
        return Rotation2d.fromRotations(m_cancoder.getPosition().getValueAsDouble());
    }

    public Rotation2d getVelocity() {
        // Encoder returns RPM
        return Rotation2d.fromRotations(m_cancoder.getVelocity().getValueAsDouble());
    }

    // debug use only
    public void run(double speed) {
        m_motor.set(speed);
    }

    // set wrist angle
    public void setAngle(Rotation2d angle) {
        m_goal = angle;
        SmartDashboard.putNumber("wrist/goal", m_goal.getDegrees());
    }
    
    // needs to be public so that commands can get the restricted angle
    public Rotation2d limitWristAngle(Rotation2d angle, double elevHeight) {
        return angle;
    }

    public boolean angleWithinTolerance() {
        return Math.abs(m_goalClipped.minus(getAngle()).getDegrees()) < ANGLE_TOLERANCE_DEG;
    }

    public void resetGoal() {
        Rotation2d angle = getAngle();
        setAngle(angle);
        m_currentState.position = angle.getRotations();
        m_currentState.velocity = 0;
    }

    public void setCoastMode() {
        boolean coastMode = SmartDashboard.getBoolean("wrist/coastMode", false);
        if (coastMode) {
            m_motor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            m_motor.stopMotor();
        } else
        m_motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}