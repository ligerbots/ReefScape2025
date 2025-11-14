// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class YAMSEndEffectorPivot extends SubsystemBase {
    
    private static final Angle MIN_ANGLE_LOW = Degrees.of(25);
    private static final Angle MAX_ANGLE_LOW = Degrees.of(302);

    private static final Angle MIN_ANGLE_HIGH = Degrees.of(-50);
    private static final Angle MAX_ANGLE_HIGH = Degrees.of(302);

    // NOTE: All constants were taken from the 2023 arm 
    // Note: Current values for limits are refrenced with the shooter being flat
    // facing fowards as zero.
    // As of writing the above note we still may want to change the limits
    public static final Angle ANGLE_TOLERANCE = Degrees.of(1.0);

    // private static final double GEAR_RATIO = 18.0/42.0/25.0;


    // position constants for commands
    // private static final double ADJUSTMENT_STEP = Math.toRadians(1.0);
    
    // 25:1 planetary plus 42:18 sprockets
    // private static final double GEAR_RATIO = 25.0 * (42.0 / 18.0);
      
    // Constants to limit the shooterPivot rotation speed
    // max vel: 1 rotation = 10 seconds  and then gear_ratio
    // private static final double MAX_VEL_ROT_PER_SEC = 1.5;
    private static final AngularVelocity MAX_VEL_ROT_PER_SEC = RotationsPerSecond.of(2.5);
    private static final AngularAcceleration MAX_ACC_ROT_PER_SEC2 = RotationsPerSecondPerSecond.of(7.5);

    // Zero point of the absolute encoder
    private static final double ABS_ENCODER_ZERO_OFFSET = 172.25/360.0;

    // Constants for the pivot PID controller
    private static final double K_P = 5.0;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;
    private static final double K_G = 1.17;

    private static final Distance ROBOT_MAX_HEIGHT = Inches.of(42);
    private static final Distance ROBOT_MAX_LENGTH = Inches.of(30);

    private static final Current CURRENT_LIMIT = Amps.of(60);
    private static final Mass MASS = Pounds.of(3); // TODO check mass
    private static final Distance LENGTH = Inches.of(18); // TODO check length

    private final SparkMax m_armMotor = new SparkMax(Constants.END_EFFECTOR_PIVOT_CAN_ID, MotorType.kBrushless);

    private final SmartMotorControllerConfig m_motorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(K_P, K_I, K_D, MAX_VEL_ROT_PER_SEC, MAX_ACC_ROT_PER_SEC2)
        .withSimClosedLoopController(K_P, K_I, K_D, MAX_VEL_ROT_PER_SEC, MAX_ACC_ROT_PER_SEC2)
        .withSoftLimit(MIN_ANGLE_HIGH, MAX_ANGLE_HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(CURRENT_LIMIT)
        .withMotorInverted(false)
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withFeedforward(new ArmFeedforward(0, K_G, 0, 0))
        .withSimFeedforward(new ArmFeedforward(0, K_G, 0, 0))
        .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController m_smartMotor = new SparkWrapper(m_armMotor, DCMotor.getNEO(1), m_motorConfig);
    
    private final MechanismPositionConfig m_robotToMechanism = new MechanismPositionConfig()
        .withMaxRobotHeight(ROBOT_MAX_HEIGHT)
        .withMaxRobotLength(ROBOT_MAX_LENGTH)
        .withRelativePosition(new Translation3d(Inches.of(0), Inches.of(0), Inches.of(30)));

    private ArmConfig m_config = new ArmConfig(m_smartMotor)
        .withLength(LENGTH)
        .withHardLimit(MIN_ANGLE_HIGH, MAX_ANGLE_HIGH)
        .withTelemetry("Pivot", TelemetryVerbosity.HIGH)
        .withMass(MASS)
        .withStartingPosition(MIN_ANGLE_LOW)
        .withMechanismPositionConfig(m_robotToMechanism);

    private final Arm m_arm = new Arm(m_config);

    private final SparkAbsoluteEncoder m_absoluteEncoder;
    private final SparkAbsoluteEncoderSim m_absoluteEncoderSim;

    private final DoubleSupplier m_elevatorHeight;

    /** Creates a new YAMSEndEffectorPivot. */
    public YAMSEndEffectorPivot(DoubleSupplier elevatorHeight) {
        m_elevatorHeight = elevatorHeight;

        m_absoluteEncoder = m_armMotor.getAbsoluteEncoder();
        m_absoluteEncoderSim = new SparkAbsoluteEncoderSim(m_armMotor);
        m_absoluteEncoderSim.setZeroOffset(ABS_ENCODER_ZERO_OFFSET);

        SmartDashboard.putNumber("pivot/testAngle", 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_arm.updateTelemetry();

        SmartDashboard.putNumber("pivot/angle", m_arm.getAngle().in(Degrees));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        m_arm.simIterate();
    }
    
    // configure the encoders, once everything is initialized
    // also set current setPoint so that it does not move immediately when enabled
    public void initPivot() {
        Rotation2d currAng = Rotation2d.fromRotations(m_absoluteEncoder.getPosition());
        m_smartMotor.setEncoderPosition(currAng.getMeasure());
        // reset the goal angle to be the current value
        setAngle(currAng);
    }

    // get the current pivot angle
    public Rotation2d getAngle() {
        return new Rotation2d(m_arm.getAngle());
    }

    private Rotation2d getTarget() {
        return new Rotation2d(m_arm.getMechanismSetpoint().orElse(Degrees.zero()));
    }

    // set shooterPivot angle
    public void setAngle(Rotation2d angle) {       
        double elevHeight = m_elevatorHeight.getAsDouble();
        Rotation2d limitedAngle = limitPivotAngle(angle, elevHeight);

        // setAngle returns a Command. It does not actually set the angle
        // arm.setAngle(limitedAngle.getMeasure());

        m_arm.getMotor().setPosition(limitedAngle.getMeasure());
    }

    public boolean isOutsideLowRange() {
        Angle angle = getAngle().getMeasure();
        return angle.lte(MIN_ANGLE_LOW) || angle.gte(MAX_ANGLE_LOW);
    }

    // needs to be public so that commands can get the restricted angle
    public Rotation2d limitPivotAngle(Rotation2d angle, double elevHeight) {
        double angleClamped;
        if (elevHeight <= Elevator.HEIGHT_LOW_RANGE)
            angleClamped = MathUtil.clamp(angle.getDegrees(), MIN_ANGLE_LOW.in(Degrees), MAX_ANGLE_LOW.in(Degrees));
        else angleClamped = MathUtil.clamp(angle.getDegrees(), MIN_ANGLE_HIGH.in(Degrees), MAX_ANGLE_HIGH.in(Degrees));
        return Rotation2d.fromDegrees(angleClamped);
    }

    public boolean angleWithinTolerance() {
        return getAngle().getMeasure().isNear(getTarget().getMeasure(), ANGLE_TOLERANCE);
    }

    public void setCoastMode() {
        boolean coastMode = SmartDashboard.getBoolean("shooterPivot/coastMode", false);
        if (coastMode)
            m_motorConfig.withIdleMode(MotorMode.COAST);
        else
            m_motorConfig.withIdleMode(MotorMode.BRAKE);
    }
}
