// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Grams;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

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
    
    private static final Angle MIN_ANGLE_LOW_DEG = Degrees.of(25);
    private static final Angle MAX_ANGLE_LOW_DEG = Degrees.of(302);

    private static final Angle MIN_ANGLE_HIGH_DEG = Degrees.of(0);
    private static final Angle MAX_ANGLE_HIGH_DEG = Degrees.of(302);

    // NOTE: All constants were taken from the 2023 arm 
    // Note: Current values for limits are refrenced with the shooter being flat
    // facing fowards as zero.
    // As of writing the above note we still may want to change the limits
    public static final Angle ANGLE_TOLERANCE = Degrees.of(1.0);


    private static final double GEAR_RATIO = 18.0/42.0/25.0;


    // position constants for commands
    // private static final double ADJUSTMENT_STEP = Math.toRadians(1.0);
    
    // 25:1 planetary plus 42:18 sprockets
    // private static final double GEAR_RATIO = 25.0 * (42.0 / 18.0);
      
    // Constants to limit the shooterPivot rotation speed
    // max vel: 1 rotation = 10 seconds  and then gear_ratio
    // private static final double MAX_VEL_ROT_PER_SEC = 1.5;
    private static final AngularVelocity MAX_VEL_ROT_PER_SEC = RotationsPerSecond.of(2.5);

    
    private static final AngularAcceleration MAX_ACC_ROT_PER_SEC2 = RotationsPerSecondPerSecond.of(7.5);
    private static final double ROBOT_LOOP_PERIOD = 0.02;

    // Zero point of the absolute encoder
    private static final double ABS_ENCODER_ZERO_OFFSET = 172.25/360.0;//142.05/360;

    // Constants for the pivot PID controller
    private static final double K_P = 5.0;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;

    private static final Distance ROBOT_MAX_HEIGHT = Inches.of(42);
    private static final Distance ROBOT_MAX_LENGTH = Inches.of(30);

    private static final Current CURRENT_LIMIT = Amps.of(60);
    private static final Mass MASS = Grams.of(0); // TODO check mass
    private static final Distance LENGTH = Meters.of(0); // TODO check length

    private static final Angle MIN_ANGLE = MIN_ANGLE_HIGH_DEG;
    private static final Angle MAX_ANGLE = MAX_ANGLE_HIGH_DEG;

    private static final MotorMode COAST_MODE = MotorMode.BRAKE;


    private final SparkMax armMotor = new SparkMax(1, MotorType.kBrushless);
    //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
    //          .withMechanismPosition()
    //          .withRotorPosition()
    //          .withMechanismLowerLimit()
    //          .withMechanismUpperLimit();
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(K_P, K_I, K_D, MAX_VEL_ROT_PER_SEC, MAX_ACC_ROT_PER_SEC2)
        .withSoftLimit(MIN_ANGLE, MAX_ANGLE)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withIdleMode(COAST_MODE)
        .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(CURRENT_LIMIT)
        .withMotorInverted(false)
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
        .withControlMode(ControlMode.CLOSED_LOOP);
    private final SmartMotorController motor = new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);
    private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
        .withMaxRobotHeight(ROBOT_MAX_HEIGHT)
        .withMaxRobotLength(ROBOT_MAX_LENGTH)
        .withRelativePosition(new Translation3d(Meters.of(0.25), Meters.of(0), Meters.of(0.5)));


    private ArmConfig m_config = new ArmConfig(motor)
        .withLength(LENGTH)
        .withHardLimit(MIN_ANGLE, MAX_ANGLE)
        .withTelemetry("ArmExample", TelemetryVerbosity.HIGH)
        .withMass(MASS)
        .withStartingPosition(Degrees.of(0))
        //.withHorizontalZero(Degrees.of(0))
        .withMechanismPositionConfig(robotToMechanism);
    private final Arm arm = new Arm(m_config);
    private final DoubleSupplier m_elevatorHeight;

    /** Creates a new YAMSEndEffectorPivot. */
    // Construct a new shooterPivot subsystem
    public YAMSEndEffectorPivot(DoubleSupplier elevatorHeight) {
        m_elevatorHeight = elevatorHeight;

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }

    // get the current pivot angle
    public Rotation2d getAngle() {
        return new Rotation2d(arm.getAngle());
    }
    public Rotation2d getTarget() {
        return new Rotation2d(arm.getMechanismSetpoint().orElse(Radians.zero()));
    }
    // // Encoder returns RPM
    // public Rotation2d getVelocity() {
    //     arm.set();
    // }
    // public void run(double speed) {
        
    // }
    // set shooterPivot angle
    public void setAngle(Rotation2d angle) {
        double elevHeight = m_elevatorHeight.getAsDouble();
        Rotation2d limitedAngle = limitPivotAngle(angle, elevHeight);
        arm.setAngle(limitedAngle.getMeasure());
    }
    public boolean isOutsideLowRange() {
        Angle angle = getAngle().getMeasure();
        return angle.lte(MIN_ANGLE_LOW_DEG) || angle.gte(MAX_ANGLE_LOW_DEG);
    }
    // needs to be public so that commands can get the restricted angle
    public Rotation2d limitPivotAngle(Rotation2d angle, double elevHeight) {
        double angleClamped;
        if (elevHeight <= Elevator.HEIGHT_LOW_RANGE)
            angleClamped = MathUtil.clamp(angle.getDegrees(), MIN_ANGLE_LOW_DEG.in(Degrees), MAX_ANGLE_LOW_DEG.in(Degrees));
        else angleClamped = MathUtil.clamp(angle.getDegrees(), MIN_ANGLE_HIGH_DEG.in(Degrees), MAX_ANGLE_HIGH_DEG.in(Degrees));
        return Rotation2d.fromDegrees(angleClamped);
    }
    public boolean angleWithinTolerance() {
        return getAngle().getMeasure().isNear(getTarget().getMeasure(), ANGLE_TOLERANCE);
    }
    public void setCoastMode() {
        boolean coastMode = SmartDashboard.getBoolean("shooterPivot/coastMode", false);
        if (coastMode) {
            motorConfig.withIdleMode(MotorMode.COAST);
        } else motorConfig.withIdleMode(COAST_MODE);
    }
    public void initPivot() {

    }
}
