// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    public static final double HEIGHT_LOW_RANGE = Units.inchesToMeters(2.0);

    private static final double GEAR_REDUCTION = 5.0;  // 5:1 planetary
    // diameter of final 18 tooth gear
    private static final double FINAL_GEAR_DIAMETER = Units.inchesToMeters(1.504); 
    // calibration: (circumference of 18T gear) * (2 for 2nd stage) / (motor gear reduction)
    private static final double METER_PER_REVOLUTION = Math.PI * FINAL_GEAR_DIAMETER * 2.0 / GEAR_REDUCTION;
    
    private static final double MAX_LENGTH_METERS = Units.inchesToMeters(63.0);
    private static final double MIN_LENGTH_METERS = Units.inchesToMeters(0.25);
    // private static int m_lengthAdjustment = 1;
    
    // Tolerance for commands
    private static final double LENGTH_TOLERANCE_METERS = Units.inchesToMeters(0.5);

    private static final double MIN_HEIGHT_TURN_OFF = Units.inchesToMeters(1.0);

    private static final double MAX_VEL_METER_PER_SEC = Units.inchesToMeters(1000.0);
    private static final double MAX_ACC_METER_PER_SEC_SQ = Units.inchesToMeters(2500.0);
    private static final double MAX_JERK_METER_PER_SEC3 = Units.inchesToMeters(12500.0);
    
    private static final int CURRENT_LIMIT = 100;
    
    private static final double OFFSET_METER = 0.0;

    private static final double GRAVITY_VOLTAGE = 0.6;
    private static final double K_P = 3.0;
    private static final double K_P_DOWN = 1.0;

    // private final int POTENTIOMETER_CHANNEL = 2; //TODO: Update with actual value
    // private final double POTENTIOMETER_RANGE_METERS = -2.625; // meters, the string potentiometer on takes in range in integers TODO: update to correct value
    // private final double POTENTIOMETER_OFFSET = 2.51; //TODO: Find inital value and update
    
    // Define the motor and encoders
    private final TalonFX m_motorLeft;
    private final TalonFX m_motorRight;

    // private final AnalogPotentiometer m_stringPotentiometer;
    
    // height goal in meters
    private double m_goalMeters = 0;
    
    private BooleanSupplier m_pivotOutsideLowRange = null;

    /** Creates a new Elevator. */
    @SuppressWarnings("removal")   // ignore deprecation warning
    public Elevator() {
        m_motorLeft = new TalonFX(Constants.ELEVATOR_LEFT_CAN_ID);
        m_motorRight = new TalonFX(Constants.ELEVATOR_RIGHT_CAN_ID);

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        
        // set slot 0 gains
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kS = 0; 
        slot0configs.kG = GRAVITY_VOLTAGE;  // overcome gravity
        slot0configs.GravityType = GravityTypeValue.Elevator_Static;
        slot0configs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
        slot0configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0configs.kP = K_P;  // start small!!!
        slot0configs.kI = 0.0; // no output for integrated error
        slot0configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

        // set slot 1 gains
        Slot1Configs slot1configs = talonFXConfigs.Slot1;
        slot1configs.kS = 0;
        slot1configs.kG = GRAVITY_VOLTAGE; // overcome gravity
        slot1configs.GravityType = GravityTypeValue.Elevator_Static;
        slot1configs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
        slot1configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
        slot1configs.kP = K_P_DOWN; // start small!!!
        slot1configs.kI = 0.0; // no output for integrated error
        slot1configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
        
        // set Motion Magic settings
        MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
        
        magicConfigs.MotionMagicCruiseVelocity = heightToRotations(MAX_VEL_METER_PER_SEC); // Target cruise velocity of 80 rps //TODO Change values TOTAL GUESSES 
        magicConfigs.MotionMagicAcceleration = heightToRotations(MAX_ACC_METER_PER_SEC_SQ); // Target acceleration of 160 rps/s (0.5 seconds)
        magicConfigs.MotionMagicJerk = heightToRotations(MAX_JERK_METER_PER_SEC3); // Target jerk of 1600 rps/s/s (0.1 seconds)
        
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withStatorCurrentLimit(CURRENT_LIMIT);
        talonFXConfigs.withCurrentLimits(currentLimits);
        
        m_motorLeft.getConfigurator().apply(talonFXConfigs);
        
        // enable brake mode (after main config)
        m_motorLeft.setNeutralMode(NeutralModeValue.Brake);
        m_motorLeft.setInverted(true);
       
        m_motorRight.getConfigurator().apply(talonFXConfigs);
        
        // enable brake mode (after main config)
        m_motorRight.setNeutralMode(NeutralModeValue.Brake);
        m_motorRight.setInverted(false);
        
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
        double goalClipped = limitElevatorLength(m_goalMeters, m_pivotOutsideLowRange.getAsBoolean());

        // // cross check that pivot is in a good place to go low
        // if (height <= HEIGHT_LOW_RANGE && m_pivotOutsideLowRange.getAsBoolean()) {
        //     setHeight(HEIGHT_LOW_RANGE);
        // }

        // if basically at the bottom, turn off the motor
        double height = getHeight();
        if (goalClipped < MIN_HEIGHT_TURN_OFF && height < MIN_HEIGHT_TURN_OFF) {
            // create 1 and re-use - save a *little* bit of garbage collection
            VoltageOut zero = new VoltageOut(0);
            m_motorLeft.setControl(zero);
            m_motorRight.setControl(zero);
        } else {
            // create 1 and re-use - save a *little* bit of garbage collection
            MotionMagicVoltage setpt = new MotionMagicVoltage(heightToRotations(goalClipped));
            // if going down, need to use Slot1
            boolean goingDown = goalClipped < height;
            setpt.Slot = goingDown ? 1 : 0;
            m_motorLeft.setControl(setpt);    
            m_motorRight.setControl(setpt);    
        }
        
        SmartDashboard.putNumber("elevator/height", Units.metersToInches(height));
        SmartDashboard.putBoolean("elevator/onGoal", lengthWithinTolerance());
        SmartDashboard.putNumber("elevator/currentGoal", 
            Units.metersToInches(rotationsToHeight(m_motorLeft.getClosedLoopReference().getValueAsDouble())));

        SmartDashboard.putNumber("elevator/leftVoltage", m_motorLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("elevator/leftSupplyCurrent", m_motorLeft.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("elevator/leftStatorCurrent", m_motorLeft.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("elevator/leftDutyCycle", m_motorLeft.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("elevator/leftVelocity", m_motorLeft.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("elevator/rightVoltage", m_motorRight.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("elevator/rightSupplyCurrent", m_motorRight.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("elevator/rightStatorCurrent", m_motorRight.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("elevator/rightDutyCycle", m_motorRight.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("elevator/rightVelocity", m_motorRight.getVelocity().getValueAsDouble());
    }
    
    // set elevator length in meters
    public void setHeight(double goal) {
        m_goalMeters = goal;
        SmartDashboard.putNumber("elevator/goal", Units.metersToInches(m_goalMeters));
    }
    
    public double getHeight() {
        // use the average from the two sides
        double avgH = 0.5 * (m_motorLeft.getPosition().getValueAsDouble() + m_motorRight.getPosition().getValueAsDouble());
        return rotationsToHeight(avgH);
        // return m_stringPotentiometer.get();  
    }

    public double getGoal () {
        return m_goalMeters;
    }
    
    public void zeroElevator() {
        m_motorLeft.setPosition(heightToRotations(OFFSET_METER));
        m_motorRight.setPosition(heightToRotations(OFFSET_METER));
        // updateMotorEncoderOffset();
    }
    
    public void resetGoal() {
        setHeight(getHeight());
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
    
    private static double limitElevatorLength(double length, boolean pivotOutsideRange) {
        return MathUtil.clamp(length, pivotOutsideRange ? HEIGHT_LOW_RANGE : MIN_LENGTH_METERS, MAX_LENGTH_METERS);
    }
}