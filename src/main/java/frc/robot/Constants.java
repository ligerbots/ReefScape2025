// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
* The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
* constants. This class should not be used for any other purpose. All constants should be declared
* globally (i.e. public static). Do not put anything functional in this class.
*
* <p>It is advised to statically import this class (or one of its inner classes) wherever the
* constants are needed, to reduce verbosity.
*/
public final class Constants {
    //1-4 reserved for swerve 
    public static final int KITBOT_ROLLER_ID = 5;
    public static final int CLIMBER_ID = 6;  // Falcon500
    public static final int END_EFFECTOR_PIVOT_CAN_ID = 9; 
    public static final int CORAL_EFFECTOR_INTAKE_ID = 7;
    public static final int ALGAE_EFFECTOR_INTAKE_ID = 8;
   
    public static final int ELEVATOR_LEFT_CAN_ID = 5;  // Falcon500
    public static final int ELEVATOR_RIGHT_CAN_ID = 12;  // Falcon500

    public static final int CORAL_GROUND_PIVOT_ID = 10; 
    public static final int CORAL_GROUND_ROLLER_ID = 11; 
    
    public static double MAX_VOLTAGE = 12.0;
    
    public enum Position {
        L1, L2, L3, L4, 
        BARGE,
        FRONT_INTAKE, BACK_INTAKE, 
        L2_ALGAE, L3_ALGAE, 
        STOW,
        PROCESSOR,
        CLIMB
    }
    
    public static boolean OUTREACH_MODE = false;
    
    // Feature flag: enable simulation in the classes
    // Can turn this off for competition to save a tiny bit of speed
    public static final boolean SIMULATION_SUPPORT = true;
    
    // if true, save the settings into the controllers after init
    public static final boolean SPARKMAX_PERSIST_PARAMETERS = false;
}
