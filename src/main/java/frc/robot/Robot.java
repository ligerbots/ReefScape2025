// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.NonZachRobotContainer;

/**
* The methods in this class are called automatically corresponding to each mode, as described in
* the TimedRobot documentation. If you change the name of this class or the package after creating
* this project, you must also update the Main.java file in the project.
*/
public class Robot extends TimedRobot {
    private Command m_autonomousCommand = null;
    private boolean m_prevIsRedAlliance = true;
    
    public static final String KITBOT_SERIAL_NUMBER = "0313baff";
    public static final String COMP_V1_SERIAL_NUMBER = "0313bb3a";
    public static final String TESTBENCH_SERIAL_NUMBER = "123";

    public enum RobotType {
        KITBOT, COMP_V1, TESTBENCH, REDESIGN
    }
    // we want this to be static so that it is easy for subsystems to query the robot type
    private static RobotType m_robotType;

    private final RobotContainer m_robotContainer;
    
    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    public Robot() {
        // // If you are trying to work with a real PV and simulation, enable this code.
        // // Otherwise it is not needed, even when running a simulation
        // if (isSimulation()) {
        //   // At-Home Network Debug Only - host the NT server on photonvision and connect to it.
        //   var ntinst = edu.wpi.first.networktables.NetworkTableInstance.getDefault();
        //   ntinst.stopServer();
        //   ntinst.setServer("photonvision.local");
        //   ntinst.startClient4("MainRobotProgram");
        // }

        // Disable the LiveWindow telemetry to lower the network load
        LiveWindow.disableAllTelemetry();

        // Enable local logging.
        // ** CAREFUL: this probably should be disabled during competition.
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        // Figure out which roboRio this is, so we know which version of the robot
        //   code to run.
        String serialNum = HALUtil.getSerialNumber();
        SmartDashboard.putString("rioSerialNumber", serialNum);
        if (serialNum.equals(KITBOT_SERIAL_NUMBER)) {
            m_robotType = RobotType.KITBOT;
        } else if (serialNum.equals(COMP_V1_SERIAL_NUMBER)) {
            m_robotType = RobotType.REDESIGN;
        } else if (serialNum.equals(TESTBENCH_SERIAL_NUMBER)) {
            m_robotType = RobotType.TESTBENCH;
        } else {
            // default to the Comp robot. Helps with simulation
            m_robotType = RobotType.COMP_V1;
        }
        SmartDashboard.putString("robotType", m_robotType.toString());

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        if (m_robotType == RobotType.KITBOT) {
            m_robotContainer = new KitbotRobotContainer();
        } else if (m_robotType == RobotType.COMP_V1) {
            m_robotContainer = new CompRobotContainer();
        } else if (m_robotType == RobotType.REDESIGN) {
            m_robotContainer = new CompRobotContainerRedesign();
       } else 
            m_robotContainer = new TestRobotContainer();
        

        // update the Coral limit switch every 2ms
        CoralEffector coralE = m_robotContainer.getCoralEffector();
        if (coralE != null) {
            addPeriodic(coralE.updateLimitSwitch(), 0.002);
        }
    }

    public static RobotType getRobotType() {
        return m_robotType;
    }

    /**
    * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
    * that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before LiveWindow and
    * SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }
    
    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}
    
    @Override
    public void disabledPeriodic() {
        m_robotContainer.resetAllGoals();

        // drivetrain might be null when testing code. So check
        DriveTrain driveTrain = m_robotContainer.getDriveTrain();

        // set drive motor to brake mode. The config change is blocking on the motors
        //   so only do this when disabled. But DriveTrain function check if the change is needed.
        // driveTrain.setBrakeMode(true);

        if (isSimulation()) {
            // YAGSL bug fix (Nov 2024)
            // The simulation needs to be told the drive speed every loop, even when disabled 
            // might be null during testing
            if (driveTrain != null) driveTrain.drive(0, 0, 0, false);
        }
        
        boolean isRedAlliance = FieldConstants.isRedAlliance();
        Command newAuto = m_robotContainer.getAutonomousCommand();
        // don't change the initialPose unless the Auto or Alliance has changed
        // don't want to override the true pose on the field (as determined by the AprilTags)
        //
        // Note: use "==" to compare autos - checks if they are the same object
        if (isRedAlliance != m_prevIsRedAlliance || newAuto != m_autonomousCommand) {
            m_autonomousCommand = newAuto;
            m_prevIsRedAlliance = isRedAlliance;
            if (driveTrain != null) driveTrain.setPose(m_robotContainer.getInitialPose());
        }
    }
    
    /** This autonomous runs the autonomous command selected by your {@link KitbotRobotContainer} class. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }
    
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }
    
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}
    
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
    
    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
    
    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}
    
    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
