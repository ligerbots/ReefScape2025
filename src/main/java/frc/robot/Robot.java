// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
* The methods in this class are called automatically corresponding to each mode, as described in
* the TimedRobot documentation. If you change the name of this class or the package after creating
* this project, you must also update the Main.java file in the project.
*/
public class Robot extends TimedRobot {
    private Command m_autonomousCommand = null;
    private boolean m_prevIsRedAlliance = true;
    
    
    private final KitbotRobotContainer m_robotContainer;
    
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

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new KitbotRobotContainer();
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
        if (isSimulation()) {
            // YAGSL bug fix (Nov 2024)
            // The simulation needs to be told the drive speed every loop, even when disabled 
            m_robotContainer.getDriveTrain().drive(0, 0, 0, false);
        }
        
        boolean isRedAlliance = FieldConstants.isRedAlliance();
        if (m_autonomousCommand == null || isRedAlliance != m_prevIsRedAlliance) {
            m_autonomousCommand = m_robotContainer.getAutonomousCommand();
            m_robotContainer.getDriveTrain().setPose(m_robotContainer.getInitialPose());
            m_prevIsRedAlliance = isRedAlliance;
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
