// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    public static final double HEIGHT_LOW_RANGE = Units.inchesToMeters(5.0);

    private static final double GEAR_REDUCTION = 15.0;  // 15:1 planetary
    // diameter of final 18 tooth gear
    private static final double FINAL_GEAR_DIAMETER = Units.inchesToMeters(1.504);  // TODO fix me
    // calibration: (circumference of 18T gear) * (2 for 2nd stage) / (motor gear reduction)
    private static final double METER_PER_REVOLUTION = Math.PI * FINAL_GEAR_DIAMETER * 2.0 / GEAR_REDUCTION;
    
    private static final double MAX_LENGTH_METERS = Units.inchesToMeters(63.0); // TODO fix me
    private static final double MIN_LENGTH_METERS = Units.inchesToMeters(0.25);
    // private static int m_lengthAdjustment = 1;
    
    // Tolerance for commands
    private static final double LENGTH_TOLERANCE_METERS = Units.inchesToMeters(0.5);

    private static final double MIN_HEIGHT_TURN_OFF = Units.inchesToMeters(1.0);

    // TODO set to good values
    private static final double MAX_VEL_METER_PER_SEC = Units.inchesToMeters(90.0);
    private static final double MAX_ACC_METER_PER_SEC_SQ = Units.inchesToMeters(110.0);
    private static final double MAX_JERK_METER_PER_SEC3 = Units.inchesToMeters(1000.0);
    
    private static final int CURRENT_LIMIT = 30;
    
    private static final double OFFSET_METER = 0.0;


  
    // private final int POTENTIOMETER_CHANNEL = 2; //TODO: Update with actual value
    // private final double POTENTIOMETER_RANGE_METERS = -2.625; // meters, the string potentiometer on takes in range in integers TODO: update to correct value
    // private final double POTENTIOMETER_OFFSET = 2.51; //TODO: Find inital value and update
    
    // Define the motor and encoders
    private final TalonFX m_motor;

    // private final AnalogPotentiometer m_stringPotentiometer;
    
    // height goal in meters
    private double m_goalMeters = 0;
    
    private BooleanSupplier m_pivotOutsideLowRange = null;

    /** Creates a new Elevator. */
    public Elevator() {
        m_motor = new TalonFX(Constants.ELEVATOR_CAN_ID);

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        
        // enable brake mode
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        
        // set slot 0 gains
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kS = 0.0; // Add 0.25 V output to overcome static friction //TODO Change values 
        slot0configs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
        slot0configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
        // m_slot0configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0configs.kP = 0.5;  // start small!!!
        slot0configs.kI = 0.0; // no output for integrated error
        slot0configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
        
        // set Motion Magic settings
        MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
        
        magicConfigs.MotionMagicCruiseVelocity = heightToRotations(MAX_VEL_METER_PER_SEC); // Target cruise velocity of 80 rps //TODO Change values TOTAL GUESSES 
        magicConfigs.MotionMagicAcceleration = heightToRotations(MAX_ACC_METER_PER_SEC_SQ); // Target acceleration of 160 rps/s (0.5 seconds)
        magicConfigs.MotionMagicJerk = heightToRotations(MAX_JERK_METER_PER_SEC3); // Target jerk of 1600 rps/s/s (0.1 seconds)
        
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(CURRENT_LIMIT);
        talonFXConfigs.withCurrentLimits(currentLimits);
        
        m_motor.getConfigurator().apply(talonFXConfigs);
        
        // No potentiometer at this time
        // m_stringPotentiometer = new AnalogPotentiometer(POTENTIOMETER_CHANNEL, POTENTIOMETER_RANGE_METERS, POTENTIOMETER_OFFSET);

        SmartDashboard.putNumber("elevator/testGoal", 0);
        zeroElevator();
    }
    
    public void setPivotCheckSupplier(BooleanSupplier pa) {
        m_pivotOutsideLowRange = pa;
    }

    @Override
    public void periodic() {
        double height = Units.metersToInches(getHeight());

        // cross check that pivot is in a good place to go low
        // if (height <= HEIGHT_LOW_RANGE && m_pivotOutsideLowRange.getAsBoolean()) {
        //     setHeight(HEIGHT_LOW_RANGE);
        // }

        // if basically at the bottom, turn off the motor
        if (m_goalMeters < MIN_HEIGHT_TURN_OFF && height < MIN_HEIGHT_TURN_OFF) {
            m_motor.setControl(new VoltageOut(0));
        }

        SmartDashboard.putNumber("elevator/height", height);
        SmartDashboard.putBoolean("elevator/onGoal", lengthWithinTolerance());
        SmartDashboard.putNumber("elevator/currentGoal", 
            Units.metersToInches(rotationsToHeight(m_motor.getClosedLoopReference().getValueAsDouble())));
        SmartDashboard.putNumber("elevator/voltage", m_motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("elevator/supplyCurrent", m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("elevator/statorCurrent", m_motor.getStatorCurrent().getValueAsDouble());
    }
    
    // set elevator length in meters
    public void setHeight(double goal) {
        m_goalMeters = limitElevatorLength(goal);
        
        MotionMagicVoltage m_request = new MotionMagicVoltage(0);        
        m_motor.setControl(m_request.withPosition(heightToRotations(m_goalMeters)));
        
        SmartDashboard.putNumber("elevator/goal", Units.metersToInches(m_goalMeters));
    }
    
    public double getHeight() {
        return rotationsToHeight(m_motor.getPosition().getValueAsDouble());
        // return m_stringPotentiometer.get();  
    }
    
    public void zeroElevator() {
        m_motor.setPosition(heightToRotations(OFFSET_METER));
        // updateMotorEncoderOffset();
    }
    
    // public double getPotentiometerReadingMeters(){
    //     return m_stringPotentiometer.get();
    // }
    
    // public void updateMotorEncoderOffset() {
    //     m_motor.setPosition(getPotentiometerReadingMeters());
    // }
    
    // public void adjustLength(boolean goUp) {
    //     double adjust = (goUp ? 1 : -1) * ADJUSTMENT_STEP;
    //     m_lengthAdjustment += adjust;
    //     setHeight(getHeight() + adjust, false);
    // }
    
    public boolean lengthWithinTolerance() {
        return Math.abs(getHeight() - m_goalMeters) < LENGTH_TOLERANCE_METERS;
    }
    
    private static double heightToRotations(double position) {
        return position / METER_PER_REVOLUTION;
    }
    
    private static double rotationsToHeight(double rotations) {
        return rotations * METER_PER_REVOLUTION;
    }
    
    private static double limitElevatorLength(double length) {
        return MathUtil.clamp(length, MIN_LENGTH_METERS, MAX_LENGTH_METERS);
    }
}