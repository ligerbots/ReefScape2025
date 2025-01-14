// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;

public class AprilTagVision extends SubsystemBase {
    // variable to turn on/off our private tag layout
    // if this is false, the compiler should remove all the unused code.
    public static final boolean USE_PRIVATE_TAG_LAYOUT = false;

    // Use the multitag pose estimator
    public static final boolean USE_MULTITAG = true;

    // Plot vision solutions
    public static final boolean PLOT_VISIBLE_TAGS = true;
    public static final boolean PLOT_POSE_SOLUTIONS = true;
    public static final boolean PLOT_ALTERNATE_POSES = true;

    // constants for extra tags in the shed lengths in meters!!)
    static final double SHED_TAG_NODE_XOFFSET = 0.45;
    static final double SHED_TAG_NODE_ZOFFSET = 0.31;
    static final double SHED_TAG_SUBSTATION_YOFFSET = 1.19;

    // private static final String CAMERA_NAME_FRONT = "ArducamFront"; // wouldn't need this if we hardcode it in Camera enum
    // private static final String CAMERA_NAME_BACK = "ArducamBack";
    // private final PhotonCamera m_aprilTagCameraFront = new PhotonCamera(CAMERA_NAME_FRONT);
    // private final PhotonCamera m_aprilTagCameraBack = new PhotonCamera(CAMERA_NAME_BACK);

    private Camera frontCamera;
    private Camera backCamera;
    private Camera[] cameras;

    private static AprilTagFieldLayout m_aprilTagFieldLayout;

    private class Camera {
        private PhotonCamera pCamera;
        private Transform3d robotToCam;
        private PhotonPoseEstimator poseEstimator;

        private Camera(String name, Transform3d robotToCam) {
            this.robotToCam = robotToCam;
            pCamera = new PhotonCamera(name);

            // In Case of Emergencies!!
            pCamera.setVersionCheckEnabled(false);

            // initialize poseEstimator
            // if there is multitag, use the corresponding strategy with reference as back up
            if (USE_MULTITAG) {
                poseEstimator = new PhotonPoseEstimator(
                    m_aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    robotToCam
                );
                poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
            } else {
                poseEstimator = new PhotonPoseEstimator(
                    m_aprilTagFieldLayout,
                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                    robotToCam
                );
            }
        }

        public void setDriverMode(Boolean mode) {
            pCamera.setDriverMode(mode);
        }
    }

    // Forward B&W camera for Apriltags
    // relative position of the camera on the robot to the robot center
    // use measurements to center of Swerve, and include offset
    // pitch is the Y angle, and it is positive down

    // Simulation support
    private VisionSystemSim m_visionSim;

    public AprilTagVision() {
        try {
            m_aprilTagFieldLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Unable to load AprilTag layout " + e.getMessage());
            m_aprilTagFieldLayout = null;
        }

        if (Constants.SIMULATION_SUPPORT) {
            // initialize a simulated camera. Must be done after creating the tag layout
            initializeSimulation();
        }

        // initialize cameras
        frontCamera = new Camera("ArducamFront", new Transform3d(
            new Translation3d(Units.inchesToMeters(0.5 - DriveTrain.ROBOT_SWERVE_OFFSET_X_INCHES), 0, Units.inchesToMeters(18.5)),
            new Rotation3d(0.0, Math.toRadians(-19.4), 0.0)
        ));

        backCamera = new Camera("ArducamBack", new Transform3d(
            new Translation3d(Units.inchesToMeters(-17.25 - DriveTrain.ROBOT_SWERVE_OFFSET_X_INCHES), 0, Units.inchesToMeters(10.0)),
            new Rotation3d(0.0, Math.toRadians(-18.0), Math.toRadians(180.0))
        ));

        cameras = new Camera[] {
            frontCamera,
            backCamera
        };
        
        // set the driver mode to false
        frontCamera.setDriverMode(false);
        backCamera.setDriverMode(false);
    }

    @Override
    public void periodic() {
        // set the driver mode to false
        // setDriverMode(false);

        SmartDashboard.putBoolean("aprilTagVision/frontCamera", frontCamera.pCamera.isConnected());
        SmartDashboard.putBoolean("aprilTagVision/backCamera", backCamera.pCamera.isConnected());
    }

    public void updateSimulation(Pose2d pose) {
        m_visionSim.update(pose);
    }

    public void updateOdometry(SwerveDrive swerve) {
        // Cannot do anything if there is no field layout
        if (m_aprilTagFieldLayout == null)
            return;

        try {
            if (PLOT_VISIBLE_TAGS) {
                plotVisibleTags(swerve.field, List.of(frontCamera.pCamera, backCamera.pCamera));
            }

            // Warning: be careful about fetching values. If cameras are not connected, you
            // get errors
            // Example: cannot fetch timestamp without checking for the camera.
            // Make sure to test!

            
            List<Pose3d> options;
            for (Camera c : cameras) {
                Optional<EstimatedRobotPose> estimate = getEstimate(swerve, c);
                if (estimate.isPresent()) {
                    List<Pose3d> ops = getOptions(c, estimate);
                    options.addAll(ops);
                }
            }

            Pose3d bestBackPose3d = new Pose3d();
            Pose3d bestFrontPose3d = new Pose3d();
            double minDistance = 1e6;

            int O = options.size();
            for (int i = 0; i < O; i++) {
                for (int j = i+1; j < O; j++) {
                    
                }
            }

            Optional<EstimatedRobotPose> frontEstimate = getEstimate(swerve, frontCamera);
            Optional<EstimatedRobotPose> backEstimate = getEstimate(swerve, backCamera);
            
            if (frontEstimate.isPresent() && backEstimate.isPresent()) {
                // Both are present
                // Create a list of Pose3d options for the front camera
                List<Pose3d> frontOptions = Camera.FRONT.getOptions();
                List<Pose3d> backOptions = Camera.BACK.getOptions();

                Pose3d bestBackPose3d = new Pose3d();
                Pose3d bestFrontPose3d = new Pose3d();
                double minDistance = 1e6;

                if (PLOT_ALTERNATE_POSES) {
                    plotAlternateSolutions(swerve.field, List.of(frontOptions, backOptions));
                }

                // compare all backposes and frontposes to each other to find correct robot pose
                for (Pose3d backPose : backOptions) {
                    for (Pose3d frontPose : frontOptions) {
                        double distance = calculateDifference(frontPose, backPose);

                        // makes the smallest difference the measurement
                        if (distance < minDistance) {
                            bestBackPose3d = backPose;
                            bestFrontPose3d = frontPose;
                            minDistance = distance;
                        }
                    }
                }

                swerve.addVisionMeasurement(bestFrontPose3d.toPose2d(), Camera.FRONT.camera.getLatestResult().getTimestampSeconds());
                swerve.addVisionMeasurement(bestBackPose3d.toPose2d(), Camera.BACK.camera.getLatestResult().getTimestampSeconds());

                if (PLOT_POSE_SOLUTIONS) {
                    plotVisionPoses(swerve.field, List.of(bestFrontPose3d.toPose2d(), bestBackPose3d.toPose2d()));
                }

                return;
            }

            if (frontEstimate.isPresent()) {
                Camera.FRONT.plotAndUpdate(frontEstimate, swerve);
            } else if (backEstimate.isPresent()) {
                Camera.BACK.plotAndUpdate(frontEstimate, swerve);
            } else {
                // no results, so clear the list in the Field
                plotVisionPoses(swerve.field, null);
                swerve.field.getObject("visionAltPoses").setPoses();
            }

            return;

        } catch (Exception e) {
            DriverStation.reportError("Error updating odometry from AprilTags " + e.getLocalizedMessage(), false);
        }
    }

    public Optional<EstimatedRobotPose> getEstimate(SwerveDrive swerve, Camera camera) {
        Pose2d robotPose = swerve.getPose();
        Optional<EstimatedRobotPose> estimate = getEstimateForCamera(camera.pCamera, camera.poseEstimator, robotPose);
        return estimate;
    }

    public List<Pose3d> getOptions(Camera camera, Optional<EstimatedRobotPose> estimate) {
        List<Pose3d> options = new ArrayList<Pose3d>();
        if (estimate.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
            // if multitag is used, add robot pose to options
            options.add(estimate.get().estimatedPose);
        } else {
            // if only one tag is visible, add all possible poses to options
            options = getAmbiguousPoses(camera.pCamera.getLatestResult(), camera.robotToCam);
        }

        return options;
    }

    public void plotAndUpdate(Camera camera, Optional<EstimatedRobotPose> estimate, SwerveDrive swerve) {
        Pose2d pose = estimate.get().estimatedPose.toPose2d();
        swerve.addVisionMeasurement(pose, camera.pCamera.getLatestResult().getTimestampSeconds());

        if (PLOT_POSE_SOLUTIONS) {
            plotVisionPose(swerve.field, pose);
        }
        if (PLOT_ALTERNATE_POSES) {
            // *** Yes, this is repeated code, and maybe that is bad.
            // But this will save some cycles if this PLOT option is turned off.
            if (estimate.get().strategy != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
                plotAlternateSolutions(swerve.field,
                        List.of(getAmbiguousPoses(camera.pCamera.getLatestResult(), camera.robotToCam)));
            } else
                swerve.field.getObject("visionAltPoses").setPose(pose);
        }
    }

    // get the tag ID closest to horizontal center of camera
    // we might want to use this to do fine adjustments on field element locations
    public int getCentralTagId() {
        // make sure camera connected
        if (!frontCamera.pCamera.isConnected())
            return -1;

        var targetResult = frontCamera.pCamera.getLatestResult();
        // make a temp holder var for least Y translation, set to first tags translation
        double minY = 1.0e6; // big number
        int targetID = -1;
        for (PhotonTrackedTarget tag : targetResult.getTargets()) { // for every target in camera
            // find id for current tag we are focusing on
            int tempTagID = tag.getFiducialId();

            // if tag has an invalid ID then skip this tag
            if (tempTagID < 1 || tempTagID > 16) {
                continue;
            }

            // get transformation to target
            Transform3d tagTransform = tag.getBestCameraToTarget();
            // get abs translation to target from transformation
            double tagY = Math.abs(tagTransform.getY());

            // looking for smallest absolute relative to camera Y
            // if abs Y translation of new tag is less then holder tag, it becomes holder
            // tag
            if (tagY < minY) {
                minY = tagY;
                targetID = tempTagID; // remember targetID
            }
        }

        return targetID;
    }

    // get the pose for a tag.
    // will return null if the tag is not in the field map (eg -1)
    public Optional<Pose2d> getTagPose(int tagId) {
        // optional in case no target is found
        Optional<Pose3d> tagPose = m_aprilTagFieldLayout.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            return Optional.empty(); // returns an empty optional
        }
        return Optional.of(tagPose.get().toPose2d());
    }

    // Private routines for calculating the odometry info

    private double calculateDifference(Pose3d x, Pose3d y) {
        return x.getTranslation().getDistance(y.getTranslation());
    }

    private static Optional<EstimatedRobotPose> getEstimateForCamera(PhotonCamera cam, PhotonPoseEstimator poseEstimator, Pose2d robotPose) {
        if (!cam.isConnected()) return Optional.empty();

        try {
            poseEstimator.setReferencePose(robotPose);
            PhotonPipelineResult res = cam.getLatestResult();
            return poseEstimator.update(res);
        } catch (Exception e) {
            // bad! log this and keep going
            DriverStation.reportError("Exception running PhotonPoseEstimator", e.getStackTrace());
            return Optional.empty();
        }
    }

    // create a strategy based off closestToReferencePoseStrategy that returns all
    // possible robot positions
    private static ArrayList<Pose3d> getAmbiguousPoses(PhotonPipelineResult result, Transform3d robotToCamera) {
        ArrayList<Pose3d> ambigiousPoses = new ArrayList<>();
        for (PhotonTrackedTarget target : result.targets) {
            int targetFiducialId = target.getFiducialId();

            // Don't report errors for non-fiducial targets. This could also be resolved by
            // adding -1 to
            // the initial HashSet.
            if (targetFiducialId == -1)
                continue;

            Optional<Pose3d> targetPosition = m_aprilTagFieldLayout.getTagPose(target.getFiducialId());

            if (targetPosition.isEmpty())
                continue;

            // add all possible robot positions to the array that is returned
            ambigiousPoses.add(
                    targetPosition.get()
                            .transformBy(target.getBestCameraToTarget().inverse())
                            .transformBy(robotToCamera.inverse()));
            ambigiousPoses.add(
                    targetPosition.get()
                            .transformBy(target.getAlternateCameraToTarget().inverse())
                            .transformBy(robotToCamera.inverse()));
        }

        return ambigiousPoses;
    }

    // private static AprilTag constructTag(int id, double x, double y, double z,
    // double angle) {
    // return new AprilTag(id, new Pose3d(x, y, z, new Rotation3d(0, 0,
    // Math.toRadians(angle))));
    // }

    // // add a new tag relative to another tag. Assume the orientation is the same
    // private static AprilTag constructTagRelative(int id, Pose3d basePose, double
    // x, double y, double z) {
    // return new AprilTag(id, new Pose3d(basePose.getX() + x, basePose.getY() + y,
    // basePose.getZ() + z, basePose.getRotation()));
    // }

    private void initializeSimulation() {
        m_visionSim = new VisionSystemSim("AprilTag");

        // roughly our Arducam camera
        SimCameraProperties prop = new SimCameraProperties();
        prop.setCalibration(800, 600, Rotation2d.fromDegrees(90.0));
        prop.setFPS(60);
        prop.setAvgLatencyMs(10.0);
        prop.setLatencyStdDevMs(3.0);

        // Note: NetworkTables does not update the timestamp of an entry if the value does not change.
        // The timestamp is used by PVLib to know if there is a new frame, so in a simulation
        // with no uncertainty, it thinks that it is not detecting a tag if the robot is static.
        // So, always have a little bit of uncertainty.
        prop.setCalibError(0.1, 0.03);

        PhotonCameraSim cam = new PhotonCameraSim(frontCamera.pCamera, prop);
        cam.setMaxSightRange(Units.feetToMeters(20.0));
        m_visionSim.addCamera(cam, frontCamera.robotToCam);

        cam = new PhotonCameraSim(backCamera.pCamera, prop);
        cam.setMaxSightRange(Units.feetToMeters(20.0));
        m_visionSim.addCamera(cam, backCamera.robotToCam);

        m_visionSim.addAprilTags(m_aprilTagFieldLayout);
    }

    // --- Routines to plot the vision solutions on a Field2d ---------

    private void plotVisionPoses(Field2d field, List<Pose2d> poses) {
        if (field == null)
            return;
        if (poses == null)
            field.getObject("visionPoses").setPoses();
        else
            field.getObject("visionPoses").setPoses(poses);
    }

    private static void plotVisionPose(Field2d field, Pose2d pose) {
        if (field == null)
            return;
        field.getObject("visionPoses").setPose(pose);
    }

    private void plotVisibleTags(Field2d field, List<PhotonCamera> cameraList) {
        if (field == null)
            return;

        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        for (PhotonCamera cam : cameraList) {
            if (!cam.isConnected()) continue;

            for (PhotonTrackedTarget target : cam.getLatestResult().getTargets()) {
                int targetFiducialId = target.getFiducialId();
                if (targetFiducialId == -1)
                    continue;

                Optional<Pose3d> targetPosition = m_aprilTagFieldLayout.getTagPose(targetFiducialId);
                if (!targetPosition.isEmpty())
                    poses.add(targetPosition.get().toPose2d());
            }
        }

        field.getObject("visibleTagPoses").setPoses(poses);
    }

    private static void plotAlternateSolutions(Field2d field, List<List<Pose3d>> allPoses) {
        if (field == null)
            return;

        ArrayList<Pose2d> both = new ArrayList<>();
        for (List<Pose3d> pl : allPoses) {
            for (Pose3d p : pl)
                both.add(p.toPose2d());
        }

        field.getObject("visionAltPoses").setPoses(both);
    }
}