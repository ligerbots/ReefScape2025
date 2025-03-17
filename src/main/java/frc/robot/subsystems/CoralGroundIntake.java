// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralGroundIntake extends SubsystemBase {
    // NOTE: All constants were taken from the 2023 arm 
    // Note: Current values for limits are refrenced with the shooter being flat
    // facing fowards as zero.
    // As of writing the above note we still may want to change the limits
    public static final double ANGLE_TOLERANCE_DEG = 1.0;

    private static final int CURRENT_LIMIT = 60;

    private static final double MIN_ANGLE = -90.0; //FIXME: Update with real values
    private static final double MAX_ANGLE = 90.0; //FIXME: Update with real values

    // position constants for commands
    // private static final double ADJUSTMENT_STEP = Math.toRadians(1.0);
    
    // 25:1 planetary plus 42:18 sprockets
    // private static final double GEAR_RATIO = 25.0 * (42.0 / 18.0);
      
    // Constants to limit the shooterPivot rotation speed
    // max vel: 1 rotation = 10 seconds  and then gear_ratio

    // Constants for the pivot PID controller
    private static final double K_P = 5.0;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;

    private final SparkMax m_motor;
    // private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_controller;


    // Construct a new shooterPivot subsystem
    public CoralGroundIntake(DoubleSupplier elevatorHeight) {
        m_motor = new SparkMax(Constants.END_EFFECTOR_PIVOT_CAN_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(CURRENT_LIMIT);
        
        // set up the PID for MAX Motion
        // config.closedLoop.pidf(K_P, K_I, K_D, K_FF);
        config.closedLoop.p(K_P).i(K_I).d(K_D);
        //  K_FF);

        config.closedLoop.outputRange(-1, 1);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.closedLoop.positionWrappingEnabled(false);  // don't treat it as a circle
        // config.closedLoop.positionWrappingInputRange(0,1.0);
                        
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // motor encoder - set calibration and offset to match absolute encoder
        // m_encoder = m_motor.getEncoder();

        // controller for PID control
        m_controller = m_motor.getClosedLoopController();
    }

    @Override
    public void periodic() {

        // double oldGoalClipped = m_goalClipped.getDegrees();

        // boolean goalChanged = Math.abs(m_goalClipped.getDegrees() - oldGoalClipped) > 5.0;
    }

    // get the current pivot angle
    public Rotation2d getPivotAngle() {
        return Rotation2d.fromRotations(m_motor.getEncoder().getPosition());
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