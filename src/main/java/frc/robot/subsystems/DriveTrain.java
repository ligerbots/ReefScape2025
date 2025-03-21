// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.config.PIDConstants;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
// import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import frc.robot.Constants;
import frc.robot.FieldConstants;

public class DriveTrain extends SubsystemBase {

    public static final double MAX_SPEED = Units.feetToMeters(15);
    
    public static final double ANGLE_TOLERANCE_RADIANS = Math.toRadians(2.0);

    private static final double DRIVE_STATOR_LIMIT = 40;

    // private static final double STEER_GEAR_RATIO = (50.0 / 14.0) * (60.0 / 10.0);

    // private static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    // This is the L2 gearing
    // private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

    public static final double ROBOT_SWERVE_OFFSET_X_INCHES = 0.0;
    private static final Translation2d ROTATION_CENTER_OFFSET = new Translation2d(Units.inchesToMeters(ROBOT_SWERVE_OFFSET_X_INCHES), 0 );

    // values from 2024 competition. Maybe should be tuned
    private static final PIDConstants PATH_PLANNER_TRANSLATION_PID = new PIDConstants(5, 0, 0);
    private static final PIDConstants PATH_PLANNER_ANGLE_PID       = new PIDConstants(5, 0, 0);
    // // local overrides for PP max values. 
    // // These are combined using "min()" with the values computed from the JSON config files, where available.
    // private static final double PATH_PLANNER_MAX_SPEED = 4.5;
    // private static final double PATH_PLANNER_MAX_ACCELERATION = 3.5;
    // private static final double PATH_PLANNER_MAX_ANGULAR_SPEED = 4.5;
    // private static final double PATH_PLANNER_MAX_ANGULAR_ACCELERATION = 4.5;

    // if true, then robot is in field centric mode
    private boolean m_fieldCentric = true;

    // if true, then robot is in precision mode
    private boolean m_precisionMode = false;

    // need to remember the configured max rotation speed
    private final double m_maxRotationSpeed;
    private static final double PRECISION_MODE_SCALE_FACTOR = 1.0 / 6.0;
    private static final double OUTREACH_MODE_SCALE_FACTOR = 0.5;
    private static final double READYTOCLIMBDEGREES = 5;
    
    // Swerve drive object
    private final SwerveDrive m_swerveDrive;

    private final AprilTagVision m_aprilTagVision;

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public DriveTrain(String jsonPath, AprilTagVision apriltagVision) {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        // Turn off for competition??
        // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        
        try {
            File jsonDir = new File(Filesystem.getDeployDirectory(), jsonPath);

            // use the conversion factors included in the JSON
            m_swerveDrive = new SwerveParser(jsonDir).createSwerveDrive(MAX_SPEED);

            // m_swerveDrive = new SwerveParser(jsonDir).createSwerveDrive(MAX_SPEED, angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        // remember configured max rotation speed
        m_maxRotationSpeed = m_swerveDrive.getMaximumChassisAngularVelocity();

        // Heading correction should only be used while controlling the robot via angle.
        m_swerveDrive.setHeadingCorrection(false);
        
        m_swerveDrive.setCosineCompensator(false);// !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation
                                                // for simulations since it causes discrepancies not seen in real life.
        
        // for now (testing!!), turn off periodic sync of the absolute encoders
        m_swerveDrive.setModuleEncoderAutoSynchronize(false, 3.0);

        CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withStatorCurrentLimit(DRIVE_STATOR_LIMIT);

        for (SwerveModule swerveModule : m_swerveDrive.getModules()) {
            ((TalonFX) swerveModule.getDriveMotor().getMotor()).getConfigurator().apply(configs);
        }

        m_aprilTagVision = apriltagVision;
        
        setupPathPlanner();
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner() {
        try {
            // Load the RobotConfig from the settings file created by GUI. 
            // You should probably store this in your Constants file
            RobotConfig config = RobotConfig.fromGUISettings();

            // TODO: fix code to allow FF
            // final boolean enableFeedforward = true;
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    // Robot pose supplier
                    this::getPose,
                    // Method to reset odometry (will be called if your auto has a starting pose)
                    this::setPose,
                    // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    this::getRobotVelocity,
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                    // optionally outputs individual module feedforwards
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        // if (enableFeedforward) {
                        //     m_swerveDrive.drive(
                        //             speedsRobotRelative,
                        //             m_swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                        //             moduleFeedForwards.linearForces());
                        // } else {
                        m_swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        // }
                    },
                    // PPHolonomicController is the built in path following controller for holonomic
                    // drive trains
                    new PPHolonomicDriveController(
                            PATH_PLANNER_TRANSLATION_PID,
                            PATH_PLANNER_ANGLE_PID),
                    // The robot configuration
                    config,
                    // whether to flip directions for Red
                    () -> FieldConstants.isRedAlliance(),
                    this
            // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Preload PathPlanner Path finding
        // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     * Inputs are deadbanded and squared.
     *
     * @param translationX     Translation [-1, 1] in the X direction. 
     * @param translationY     Translation [-1, 1] in the Y direction. 
     * @param angularRotation  Angular velocity [-1, 1] of the robot to set. 
     * @param robotCentric     Robot centric drive if true.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotation, BooleanSupplier robotCentric) {
        return run(() -> {
            drive(translationX.getAsDouble(), translationY.getAsDouble(), angularRotation.getAsDouble(), robotCentric.getAsBoolean());
        });
    }

    /**
     * Drive the robot using translative values and heading as angular velocity.
     * Inputs are deadbanded and squared.
     *
     * @param translationX     Translation [-1, 1] in the X direction. 
     * @param translationY     Translation [-1, 1] in the Y direction. 
     * @param angularRotation  Angular velocity [-1, 1] of the robot to set. 
     * @param robotCentric     Robot centric drive if true.
     */
    public void drive(double translationX, double translationY, double angularRotation, boolean robotCentric) {
        // field centric: flip direction if we are Red
        double flipDirection = FieldConstants.isRedAlliance() ? -1.0 : 1.0;

        double v_x = flipDirection * translationX * m_swerveDrive.getMaximumChassisVelocity();
        double v_y = flipDirection * translationY * m_swerveDrive.getMaximumChassisVelocity();
        double v_ang = angularRotation * m_swerveDrive.getMaximumChassisAngularVelocity();

        driveWithSpeeds(v_x, v_y, v_ang, robotCentric);
    }

    /**
     * Drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation [-1, 1] in the X direction.
     * @param translationY Translation [-1, 1] in the Y direction.
     * @param heading      Target heading in radians.
     */
    public void driveWithHeading(double translationX, double translationY, double heading) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
        // correction for this kind of control.

        // for now, only allow field-centric
        double flipDirection = FieldConstants.isRedAlliance() ? -1.0 : 1.0;

        double v_x = flipDirection * translationX * m_swerveDrive.getMaximumChassisVelocity();
        double v_y = flipDirection * translationY * m_swerveDrive.getMaximumChassisVelocity();

        // use PID in SwerveController to compute desired angular velocity
        double v_ang = m_swerveDrive.swerveController.headingCalculate(m_swerveDrive.getOdometryHeading().getRadians(), heading);

        driveWithSpeeds(v_x, v_y, v_ang, false);
    }

    /** 
     * Basic drive routine. Take individual speeds, and command the robot.
     * Includes standard offset of the center, and can include acceleration limits
     * 
     * @param speedX - speed in X direction (m/s)
     * @param speedY - speed in X direction (m/s)
     * @param speedAng - angular rotation speed (rad/s)
     * @param robotCentric - if true, x/y speeds are robot-centric
    */
    public void driveWithSpeeds(double speedX, double speedY, double speedAng, boolean robotCentric) {
        // SwerveDrive::drive() uses robot-relative speeds
        ChassisSpeeds speeds;
        if (robotCentric) {
            speeds = new ChassisSpeeds(speedX, speedY, speedAng);
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedAng, m_swerveDrive.getOdometryHeading());
        }

        // TODO: add in acceleration controls

        m_swerveDrive.drive(speeds, false, ROTATION_CENTER_OFFSET);
    }

    /**
     * Get the path follower with events.
     *
     * @param path PathPlanner path.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command followPath(PathPlannerPath path) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }

    public Command pathFindToPose(Pose2d targetPose, PathConstraints constraints) {
        
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    public static PathPlannerPath loadPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return path;
        } catch (Exception e) {
            DriverStation.reportError(String.format("Unable to load PP path %s", pathName), true);
        }
        return null;
    }

    // for the beginning of auto rountines
    public void resetDrivingModes() {
        setFieldCentricMode(true);
        setPrecisionMode(false);
    }

    // toggle whether driving is field-centric
    public void toggleFieldCentric() {
        setFieldCentricMode(!m_fieldCentric);
    }

    // toggle precision mode for driving
    public void togglePrecisionMode() {
        setPrecisionMode(!m_precisionMode);
    }

    public void setFieldCentricMode(boolean fieldCentricMode) {
        m_fieldCentric = fieldCentricMode;
    }

    public void setPrecisionMode(boolean precisionMode) {
        m_precisionMode = precisionMode;
        double maxSpeed = MAX_SPEED;
        double maxAngSpeed = m_maxRotationSpeed;
        
        if (precisionMode) {
            maxSpeed *= PRECISION_MODE_SCALE_FACTOR;
            maxAngSpeed *= PRECISION_MODE_SCALE_FACTOR;
        } else if (Constants.OUTREACH_MODE) {
            maxSpeed *= OUTREACH_MODE_SCALE_FACTOR;
            maxAngSpeed *= OUTREACH_MODE_SCALE_FACTOR;
        }
        m_swerveDrive.setMaximumAllowableSpeeds(maxSpeed, maxAngSpeed);
    }

    @Override
    public void periodic() {
        // Have the vision system update based on the Apriltags, if seen
        // need to add the pipeline result
        m_aprilTagVision.updateOdometry(m_swerveDrive);
        
        SmartDashboard.putBoolean("driveTrain/readyToClimb", readyToClimb());
        SmartDashboard.putNumber("driveTrain/pitch", getPitch().getDegrees());
        SmartDashboard.putNumber("driveTrain/yaw", getYaw().getDegrees());
    }

    @Override
    public void simulationPeriodic() {
        m_aprilTagVision.updateSimulation(m_swerveDrive);
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    // public SwerveDriveKinematics getKinematics() {
    //     return swerveDrive.kinematics;
    // }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this method. 
     * However, if either gyro angle or module position is reset, this must
     * be called in order for odometry to keep working.
     *
     * @param pose The pose to set the odometry to
     */
    public void setPose(Pose2d pose) {
        m_swerveDrive.resetOdometry(pose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return m_swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        // System.out.println("setChassisSpeeds: pose " + getPose() + " speeds " + chassisSpeeds + " currVel " + getRobotVelocity());
        m_swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Display a trajectory on the field.
     *
     * @param trajectory The trajectory to display.
     */
    public void postTrajectory(Trajectory trajectory) {
        m_swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroHeading() {
        m_swerveDrive.zeroGyro();
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setBrakeMode(boolean brake) {
        m_swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to
     * resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return m_swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return m_swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return m_swerveDrive.swerveController;
    }

    public boolean readyToClimb(){
        return Math.abs(getPitch().getDegrees()) >= READYTOCLIMBDEGREES;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return m_swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        m_swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return m_swerveDrive.getPitch();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getRoll() {
        return m_swerveDrive.getRoll();
    }
     public Rotation2d getYaw() {
        return m_swerveDrive.getYaw();
     }


    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new Config(),
                        this, m_swerveDrive, 12, false),
                3.0, 5.0, 3.0);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new Config(),
                        this, m_swerveDrive),
                3.0, 5.0, 3.0);
    }
}
