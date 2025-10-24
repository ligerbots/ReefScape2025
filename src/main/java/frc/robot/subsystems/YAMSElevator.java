// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class YAMSElevator extends SubsystemBase {
  private static final double GEAR_REDUCTION = 5.0; // 5:1 planetary
  // diameter of final 18 tooth gear
  private static final double FINAL_GEAR_DIAMETER = Units.inchesToMeters(1.504);
  // calibration: (circumference of 18T gear) * (2 for 2nd stage) / (motor gearreduction)
  // private static final double METER_PER_REVOLUTION = Math.PI * FINAL_GEAR_DIAMETER * 2.0 / GEAR_REDUCTION;

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Mechanism Circumference is the distance traveled by each mechanism rotation
      // converting rotations to meters.
      .withMechanismCircumference(Meters.of(Math.PI * FINAL_GEAR_DIAMETER))
      // Feedback Constants (PID Constants)
      .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
      .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
      // Feedforward Constants
      .withFeedforward(new ElevatorFeedforward(0, 0, 0))
      .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which
      // corresponds to the gearbox attached to your motor.
      .withGearing(new MechanismGearing(GearBox.fromStages("5:1")))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFollowers(Pair.of(new TalonFX(Constants.ELEVATOR_RIGHT_CAN_ID), false));

  // Vendor motor controller object
  // private SparkMax spark = new SparkMax(4, MotorType.kBrushless);
  private final TalonFX m_motorLeft = new TalonFX(Constants.ELEVATOR_LEFT_CAN_ID);
  // private final TalonFX m_motorRight = new TalonFX(Constants.ELEVATOR_RIGHT_CAN_ID);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new TalonFXWrapper(m_motorLeft, DCMotor.getKrakenX60Foc(2), smcConfig);
  //new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
      .withStartingHeight(Meters.of(0.5))
      .withHardLimits(Meters.of(0), Meters.of(3))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(16));

  // Elevator Mechanism
  private Elevator elevator = new Elevator(elevconfig);

  /**
   * Set the height of the elevator.
   * 
   * @param angle Distance to go to.
   */
  public Command setHeight(Distance height) {
    return elevator.setHeight(height);
  }

  /**
   * Move the elevator up and down.
   * 
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) {
    return elevator.set(dutycycle);
  }

  /**
   * Run sysId on the {@link Elevator}
   */
  public Command sysId() {
    return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  /** Creates a new ExampleSubsystem. */
  public YAMSElevator() {
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    elevator.simIterate();
  }
}
