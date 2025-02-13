package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
    
    private final TalonFX m_climberMotor;
    // private final RelativeEncoder m_climberMotorEncoder;
    
    // Constants to be used in this class
    private static final double DEPLOYED_ROTATIONS = -10.0;
    private static final double CLIMB_ROTATIONS = 610.0; 
    
    // Protection values
    private static final double MAX_ROTATIONS_ALLOWED = 710.0;
    private static final double MAX_CURRENT = 100.0;

    // Current limit in the motor
    private static final double CURRENT_LIMIT = 40;

    // Winch motor speed values
    private static final double IDLE_SPEED = -0.01;
    private static final double DEPLOY_MAX_SPEED = -0.2;
    private static final double DEPLOY_MIN_SPEED = -0.1;
    private static final double CLIMB_SPEED = 0.5;

    public static final double MANUAL_SPEED = 0.3;

    private static final double EXTEND_SLOWDOWN_INTERVAL = 30.0;
    
    // State definitions:
    // IDLE - winch holding system in place, should be used for entire match until End Game.
    // DEPLOYING - winch unwinding ropes to allow climber to raise.
    // WAITING - waiting for robot to get close enough to cage to climb.
    // CLIMBING - climb. Stop automatically when we think we are high enough (based on rope length)
    // HOLDING - Normally, this would mean the the robot is now off the floor and the ratchets are engaged.
    //      Allow the driver to pull a little further, within reason
    private enum ClimberState {IDLE, DEPLOYING, WAITING, CLIMBING, HOLDING};
    private ClimberState m_climberState = ClimberState.IDLE;
    
    public Climber() {
        m_climberMotor = new TalonFX(Constants.CLIMBER_ID);
        m_climberMotor.setInverted(true);
        
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        MotorOutputConfigs mco = new MotorOutputConfigs();
        mco.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.withMotorOutput(mco);

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(CURRENT_LIMIT);
        talonFXConfigs.withCurrentLimits(currentLimits);
        
        m_climberMotor.getConfigurator().apply(talonFXConfigs);
    }
    
    @Override
    public void periodic() {
        double position = getPosition();
        double climberCurrrent = m_climberMotor.getSupplyCurrent().getValueAsDouble();
        
        SmartDashboard.putNumber("climber/supplyCurrent", climberCurrrent);
        SmartDashboard.putNumber("climber/statorCurrent", m_climberMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("climber/speed", m_climberMotor.get());
        SmartDashboard.putNumber("climber/position", position);
        SmartDashboard.putString("climber/state", m_climberState.toString());
        
        // While idle, we want a small voltage applied to hold the hooks in place.
        if (m_climberState == ClimberState.IDLE) {
            // m_climberMotor.set(IDLE_MOTOR_SPEED);
            // Note that the DEPLOYING state is entered via a command, so here we just stay IDLE
        }
        else if (m_climberState == ClimberState.DEPLOYING) {
            // Deploy goes NEGATIVE!!
            if (position <= DEPLOYED_ROTATIONS) {
                // Stop winch
                m_climberMotor.set(0.0);
                m_climberState = ClimberState.WAITING;
            }
        }
        else if (m_climberState == ClimberState.WAITING) {
            // Nothing to do here
        }
        else if (m_climberState == ClimberState.CLIMBING) {
            // The CLIMBING state is only entered via a command
            // m_climberMotor.set(CLIMB_SPEED);
        }
        else if (m_climberState == ClimberState.CLIMBING) {          
            if (position >= CLIMB_ROTATIONS) {
                m_climberMotor.set(0.0);
                m_climberState = ClimberState.HOLDING;
            }
        }
        
        // Protection: stop if position is at max or current is too high
        // Test no matter what State we are in. 
        // Do this at the end to override any settings
        if (position >= MAX_ROTATIONS_ALLOWED || climberCurrrent > MAX_CURRENT)
            m_climberMotor.set(0.0);
    }
    
    private double limitWinch(double speed) {
        if (getPosition() >= MAX_ROTATIONS_ALLOWED)
            return 0.0;
        return speed;
    }
    
    public void run(double climbSpeed) {
        m_climberMotor.set(limitWinch(climbSpeed));
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
        run(DEPLOY_MAX_SPEED);
    }
    
    public void climb() {
        m_climberState = ClimberState.CLIMBING;
        run(CLIMB_SPEED);
    }
    
    public void hold() {
        m_climberState = ClimberState.HOLDING;
        m_climberMotor.set(0.0);
    }
}
