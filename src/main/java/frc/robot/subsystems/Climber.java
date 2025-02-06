package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;

public class Climber extends SubsystemBase {

    private final SparkMax m_climberMotor;
    private final RelativeEncoder m_climberMotorEncoder;

    private boolean m_deployed = false;
    private boolean m_complete = false;

    // Constants to be used in this class
    // private static final double WINCH_GEAR_RATIO = 1.0/15.0;
    // private static final double NOMINAL_WINCH_DIAMETER = Units.inchesToMeters(0.75);
    // private static final double NOMINAL_INCHES_PER_ROTATION = Math.PI * NOMINAL_WINCH_DIAMETER * WINCH_GEAR_RATIO;
    private static final double DEPLOYED_ROTATIONS_ABOVE_INITIAL_POSITION = 190.0;//taken from last year, probrobly negative 
    // private static final double CLIMB_ROTATIONS_FINAL = 225.0;
    private static final double CLIMB_ROTATIONS_AFTER_DEPLOY= 60.0;//also take from last year 
    

    // Protection values
    private static final double MAX_WINCH_ROTATIONS_ALLOWED = 365.0;
    private static final double MAX_WINCH_CURRENT = 100.0;
    // Current limit in the SparkMax
    private static final int WINCH_CURRENT_LIMIT = 40;
    // Winch motor speed values
    private static final double IDLE_MOTOR_SPEED = -0.01;
    private static final double WINCH_EXTEND_MAX_SPEED = 1.0;
    private static final double WINCH_EXTEND_MIN_SPEED = 0.2;
    private static final double WINCH_RETRACT_SPEED = 1.0;
    public static final double WINCH_MANUAL_SPEED = 0.3;
    private static final double WINCH_CLIMB_SPEED = 0.5;

    private static final double EXTEND_SLOWDOWN_INTERVAL = 30.0;

    // speed adjustment amount if Roll is off by ROLL_ANGLE_TOLERANCE
    private static final double WINCH_CLIMB_ADJUST_SPEED = 0.1;
        private static final double MOTOR_VOLTAGE_COMP = 12;
    
        // State definitions:
        // IDLE - Winches holding hooks in place, Should be used for entire match until End Game.
        // DEPLOYING - Winches unwinding ropes to allow hooks to raise.
        // WAITING - Hooks at max height. Waiting dor robot to get close enough to chain to lower hooks.
        // RETRACTING_HOOKS - Winches start winding up the rope to lower the hooks onto the chain.
        // CLIMBING - Hooks have engaged with chain. As the winches retract further, the robot goes up.
        //      Since the robot is no longer onthe ground, we need to adjust the climb speeds to keep the robot level.
        // HOLDING - Normally, this would mean the the robot is now off the floor and the ratchets are engaged.
        //      In an emergency, if the robot rools too far, we will stop the motors and let the ratchets hold the robot.
        //      In this state, the runWinch() operation can allow further adjustments as needed.
        private enum ClimberState {IDLE, DEPLOYING, WAITING, CLIMBING, HOLDING};
        private ClimberState m_climberState = ClimberState.IDLE;
    
        private final DriveTrain m_driveTrain;
        
        public Climber(DriveTrain drivetrain) {
            m_driveTrain = drivetrain;
    
            m_climberMotor = new SparkMax(Constants.CLIMBER_ID, MotorType.kBrushless);
    
            SparkMaxConfig config = new SparkMaxConfig();
            
            config.voltageCompensation(MOTOR_VOLTAGE_COMP);
            config.smartCurrentLimit(WINCH_CURRENT_LIMIT);
            config.inverted(false);//guess 
            m_climberMotor.setCANTimeout(250);


            m_climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            m_climberMotorEncoder = m_climberMotor.getEncoder();
            m_climberMotorEncoder.setPosition(0.0);

        
    }

    @Override
    public void periodic() {
        double position = m_climberMotorEncoder.getPosition();

        double climberCurrrent = m_climberMotor.getOutputCurrent();

        // SmartDashboard.putNumber("climber/leftRPM", m_leftEncoder.getVelocity());
        // SmartDashboard.putNumber("climber/rightRPM", m_rightEncoder.getVelocity());
        SmartDashboard.putNumber("climber/Speed", m_climberMotor.get());

        SmartDashboard.putNumber("climber/Position", position);


        SmartDashboard.putNumber("climber/Current", climberCurrrent);

        SmartDashboard.putNumber("climber/pitch", m_driveTrain.getPitch().getDegrees());
        SmartDashboard.putString("climber/state", m_climberState.toString());


        // While idle, we want a small voltage applied to hold the hooks in place.
        if (m_climberState == ClimberState.IDLE) {
            m_climberMotor.set(IDLE_MOTOR_SPEED);
            // Note that the DEPLOYING state is entered via a command, so here we just stay IDLE
        }
        else if (m_climberState == ClimberState.DEPLOYING) {
            // use a "trapezoidal" profile to slow down near the end

            // If the left hook is all the way out...
            if (position >= DEPLOYED_ROTATIONS_ABOVE_INITIAL_POSITION) {
                // Stop left winch
                m_climberMotor.set(0.0);
                m_deployed = true;
            }
            else {
                // slow down over the last 20 rotations
                double speed = (DEPLOYED_ROTATIONS_ABOVE_INITIAL_POSITION - position) / EXTEND_SLOWDOWN_INTERVAL * WINCH_EXTEND_MAX_SPEED;
                m_climberMotor.set(MathUtil.clamp(speed, WINCH_EXTEND_MIN_SPEED, WINCH_EXTEND_MAX_SPEED));
            }

            // If both hooks are all the way up, wait to retract
            if (m_deployed ) {
                m_climberState = ClimberState.WAITING;
            }
        }
        else if (m_climberState == ClimberState.WAITING) {
            // Nothing to do here since both winches are stopped and the hooks are as high as they can go.
            // The RETRACTING_HOOKS state is only entered via a command
        }
        else if (m_climberState == ClimberState.CLIMBING) {
            // Here we retract both hooks. When one hook grabs the chain before the other, the robot will start to roll
            // Roll angle is positive if the left side is higher than the right side.
            // Once one side goes high, we need to stop that winch until the other hook "catches up"
            // Note that when we entered this state, both winches started retracting. We only need to stop them one at a time until
            // the robot is level again and then we will go to the CLIMBING state

            // Also check for position going well past the Raise position. This fixes the case
            //   where the robot goes straight up, never tilts. Also helps in testing not on the chain.


            // If both deployed, then we start climbing
            if (m_deployed) {
                m_climberState = ClimberState.CLIMBING;
                m_climberMotor.set(WINCH_CLIMB_SPEED);
            }
        }
        else if (m_climberState == ClimberState.CLIMBING) {

            final double adjustSpeed = WINCH_CLIMB_ADJUST_SPEED;

            // Left side
            if (position >= CLIMB_ROTATIONS_AFTER_DEPLOY || position > MAX_WINCH_ROTATIONS_ALLOWED) {
                m_climberMotor.set(0.0);
                m_complete = true;
            } else { // if (Math.abs(rollAngle) > ROLL_ANGLE_TOLERANCE) { 
                // If not done, adjust speed according to roll
                // adjustSpeed > 0 means left is high, so slow left down
                m_climberMotor.set(MathUtil.clamp(WINCH_CLIMB_SPEED - adjustSpeed, 0, 1));
            }

            // If both hooks are in enough, just hold where we are.
            if (m_complete) {
                m_climberState = ClimberState.HOLDING;
            }
        }

        // Protection: stop if position is at max or current is too high
        // Test no matter what State we are in. 
        // Do this at the end to override any settings
        if (position >= MAX_WINCH_ROTATIONS_ALLOWED || climberCurrrent > MAX_WINCH_CURRENT)
            m_climberMotor.set(0.0);
    
    }

    private double limitWinch(RelativeEncoder encoder, double speed) {
        if (encoder.getPosition() >= MAX_WINCH_ROTATIONS_ALLOWED) {
            return 0.0;
        } else
            return speed;
    }

    public void run(double climbSpeed) {
        m_climberMotor.set(limitWinch(m_climberMotorEncoder, climbSpeed));
    }

    public double getPosition(){
        return m_climberMotorEncoder.getPosition();
    }



    public double getVelocity(){
        return m_climberMotorEncoder.getVelocity();
    }

 
    public void deploy() {
        m_climberState = ClimberState.DEPLOYING;
        run(WINCH_EXTEND_MAX_SPEED);
    }

    public void climb() {
        m_climberState = ClimberState.CLIMBING;
        run(WINCH_RETRACT_SPEED);
    }

    public void hold() {
        m_climberState = ClimberState.HOLDING;
        m_climberMotor.set(0.0);
    }
}
