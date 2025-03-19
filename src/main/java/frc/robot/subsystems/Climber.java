package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
    
    private final TalonFX m_climberMotor;
    // private final RelativeEncoder m_climberMotorEncoder;
    
    // Constants to be used in this class
    private static final double DEPLOYED_ROTATIONS = -30.0;
    private static final double CLIMB_ROTATIONS = 200; 
    
    // Protection values
    private static final double MAX_ROTATIONS_ALLOWED = 220.0;
    private static final double MAX_CURRENT = 100.0;

    // Current limit in the motor
    private static final double CURRENT_LIMIT = 40;

    // Motion Magic limits
    // maxV=100, maxA=200 would be about 2.5 sec to climb
    private static final double MAX_VEL_ROT_PER_SEC = 50.0;
    private static final double MAX_ACC_ROT_PER_SEC_SQ = 100.0;
    private static final double MAX_JERK_ROT_PER_SEC3 = 2000.0;

    // Winch motor speed values
    // private static final double DEPLOY_MAX_SPEED = -0.4;
    // private static final double CLIMB_SPEED = 0.8;

    public static final double MANUAL_SPEED = 0.4;
    
    private static final double K_P = 1.0;

    // State definitions:
    // IDLE - winch holding system in place, should be used for entire match until End Game.
    // DEPLOYING - winch unwinding ropes to allow climber to raise.
    // WAITING - waiting for robot to get close enough to cage to climb.
    // CLIMBING - climb. Stop automatically when we think we are high enough (based on rope length)
    // HOLDING - Normally, this would mean the the robot is now off the floor and the ratchets are engaged.
    //      Allow the driver to pull a little further, within reason
    private enum ClimberState {IDLE, DEPLOYING, DEPLOYED, CLIMBING, HOLDING};
    private ClimberState m_climberState = ClimberState.IDLE;
    
    private double m_goal = 0;

    public Climber() {
        m_climberMotor = new TalonFX(Constants.CLIMBER_ID);

        // set config to factory default
        m_climberMotor.getConfigurator().apply(new TalonFXConfiguration());
        
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        MotorOutputConfigs mco = talonFXConfigs.MotorOutput;
        mco.NeutralMode = NeutralModeValue.Brake;
        mco.Inverted = InvertedValue.CounterClockwise_Positive;

        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        
        // set slot 0 gains
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kP = K_P;  // start small!!!
        slot0configs.kI = 0.0; // no output for integrated error
        slot0configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
        magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_ROT_PER_SEC;
        magicConfigs.MotionMagicAcceleration = MAX_ACC_ROT_PER_SEC_SQ;
        magicConfigs.MotionMagicJerk = MAX_JERK_ROT_PER_SEC3;

        m_climberMotor.getConfigurator().apply(talonFXConfigs);
        m_climberMotor.setPosition(0);
    }
    
    @Override
    public void periodic() {
        
        // not sure if this is needed
        if (motionMagicIsFinished() 
            && (m_climberState == ClimberState.DEPLOYING || m_climberState == ClimberState.CLIMBING)) {
                hold();
        }

        // if (m_climberState == ClimberState.DEPLOYING) {
        //     // Deploy goes NEGATIVE!!
        //     if (position <= DEPLOYED_ROTATIONS) {
        //         // Stop winch
        //         m_climberMotor.set(0.0);
        //         m_climberState = ClimberState.DEPLOYED;
        //     }
        // }
        // else if (m_climberState == ClimberState.CLIMBING) {          
        //     if (position >= CLIMB_ROTATIONS) {
        //         // m_climberMotor.set(0.0);

        //         // create a position closed-loop request, voltage output, slot 0 configs
        //         m_climberMotor.setControl(new PositionVoltage(getPosition()).withSlot(0));

        //         m_climberState = ClimberState.HOLDING;
        //     }
        // }

        double position = getPosition();
        double climberCurrent = m_climberMotor.getSupplyCurrent().getValueAsDouble();

        // Protection: stop if position is at max or current is too high
        // Test no matter what State we are in. 
        // Do this at the end to override any settings
        if (position >= MAX_ROTATIONS_ALLOWED || climberCurrent > MAX_CURRENT)
            m_climberMotor.set(0.0);

        SmartDashboard.putNumber("climber/supplyCurrent", climberCurrent);
        SmartDashboard.putNumber("climber/statorCurrent", m_climberMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("climber/speed", m_climberMotor.get());
        SmartDashboard.putNumber("climber/position", position);    
        SmartDashboard.putString("climber/state", m_climberState.toString());
    }
    
    private double limitWinch(double speed) {
        if (getPosition() >= MAX_ROTATIONS_ALLOWED)
            return 0.0;
        return speed;
    }
    
    public void run(double speed) {
        m_climberMotor.set(limitWinch(speed));
        // m_climberMotor.set( climbSpeed);
    }
    
    public double getPosition(){
        return m_climberMotor.getPosition().getValueAsDouble();
    }
    
    public double getVelocity(){
        return m_climberMotor.getVelocity().getValueAsDouble();
    }
    
    public void deploy() {
        m_climberState = ClimberState.DEPLOYING;
        // run(DEPLOY_MAX_SPEED);
        m_goal = DEPLOYED_ROTATIONS;
        m_climberMotor.setControl(new MotionMagicVoltage(m_goal));

    }
    
    public void climb() {
        m_climberState = ClimberState.CLIMBING;
        // run(CLIMB_SPEED);
        m_goal = CLIMB_ROTATIONS;
        m_climberMotor.setControl(new MotionMagicVoltage(m_goal));
    }
    
    public void hold() {
        m_climberState = ClimberState.HOLDING;
        // m_climberMotor.set(0.0);
        m_goal = getPosition();
        m_climberMotor.setControl(new PositionVoltage(m_goal));
    }

    boolean motionMagicIsFinished(){
        return Math.abs(m_climberMotor.getClosedLoopReference().getValueAsDouble() - m_goal) < 0.01;
    }

    public boolean isDeployed() {
        // be generous. Only used in feedback rumble
        return true; //m_climberState != ClimberState.IDLE;
    }

    public void resetGoal() {
        m_climberMotor.setControl(new MotionMagicVoltage(getPosition()));
    }
}
