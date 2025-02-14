// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.UncheckedIOException;
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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class AprilTagVision extends SubsystemBase {
    // variable to turn on/off our private tag layout
    // if this is false, the compiler should remove all the unused code.
    static final boolean USE_PRIVATE_TAG_LAYOUT = false;

    // Use the multitag pose estimator
    static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    // static final PoseStrategy POSE_STRATEGY = PoseStrategy.CLOSEST_TO_REFERENCE_POSE;
    
    // Plot vision solutions
    static final boolean PLOT_VISIBLE_TAGS = true;
    static final boolean PLOT_POSE_SOLUTIONS = true;
    static final boolean PLOT_ALTERNATE_POSES = true;

    // constants for extra tags in the shed lengths in meters!!)
    static final double SHED_TAG_NODE_XOFFSET = 0.45;
    static final double SHED_TAG_NODE_ZOFFSET = 0.31;
    static final double SHED_TAG_SUBSTATION_YOFFSET = 1.19;

    // Base standard deviations for vision results
    static final Matrix<N3, N1> SINGLE_TAG_BASE_STDDEV = VecBuilder.fill(4, 4, 8);
    static final Matrix<N3, N1> MULTI_TAG_BASE_STDDEV = VecBuilder.fill(0.5, 0.5, 1);
    static final Matrix<N3, N1> INFINITE_STDDEV = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    private enum Cam {
        FRONT(0),
        BACK(1);

        int idx;
        Cam(int idx) { this.idx = idx; }
    }

    private class Camera {
        PhotonCamera photonCamera;
        Transform3d robotToCam;
        PhotonPoseEstimator poseEstimator;
        List<PhotonPipelineResult> pipeResults;

        private Camera(String name, Transform3d robotToCam) {
            this.robotToCam = robotToCam;
            photonCamera = new PhotonCamera(name);

            // In Case of Emergencies!!
            // pCamera.setVersionCheckEnabled(false);

            // Use the standard PoseStrategy
            // Setting a fallback strategy is needed for MultiTag, and no harm for others
            poseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, POSE_STRATEGY, robotToCam);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

            // set the driver mode to false
            photonCamera.setDriverMode(false);
        }

        void setDriverMode(Boolean mode) {
            photonCamera.setDriverMode(mode);
        }
    }

    private Camera[] m_cameras;

    private AprilTagFieldLayout m_aprilTagFieldLayout;

    // Simulation support
    private VisionSystemSim m_visionSim;

    public AprilTagVision() {
        try {
            m_aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        } catch (UncheckedIOException e) {
            System.out.println("Unable to load AprilTag layout " + e.getMessage());
            m_aprilTagFieldLayout = null;
        }

        // initialize cameras
        m_cameras = new Camera[2];

        // Kitbot
        // m_cameras[Cam.FRONT.idx] = new Camera("ArducamFront", new Transform3d(
        //     new Translation3d(Units.inchesToMeters(14.5), 0, Units.inchesToMeters(11.75)),
        //     new Rotation3d(0.0, Math.toRadians(0.0), 0.0)
        // ));

        // m_cameras[Cam.BACK.idx] = new Camera("ArducamBack", new Transform3d(
        //     new Translation3d(Units.inchesToMeters(-(27.5/2 - 1.0)), 0, Units.inchesToMeters(17.0)),
        //     new Rotation3d(0.0, Math.toRadians(0.0), Math.toRadians(180.0))
        // ));
        
        // Comp Feb 8
        m_cameras[Cam.FRONT.idx] = new Camera("ArducamBack", new Transform3d(
            new Translation3d(Units.inchesToMeters(9.963562), Units.inchesToMeters(-9.550828), Units.inchesToMeters(10.402084)),
            new Rotation3d(0.0, Math.toRadians(25), 0)
                .rotateBy(new Rotation3d(0, 0, Math.toRadians(12.5)))
        ));

        m_cameras[Cam.BACK.idx] = new Camera("ArducamFront", new Transform3d(
            new Translation3d(Units.inchesToMeters(9.963562), Units.inchesToMeters(9.550828), Units.inchesToMeters(10.40208)),
            new Rotation3d(0.0, Math.toRadians(25), 0)
                .rotateBy(new Rotation3d(0, 0, Math.toRadians(-12.5)))
            ));

        if (Constants.SIMULATION_SUPPORT) {
            // initialize a simulated camera. Must be done after creating the tag layout
            initializeSimulation();
        }
    }

    @Override
    public void periodic() {
        // set the driver mode to false
        // setDriverMode(false);

        SmartDashboard.putBoolean("aprilTagVision/frontCamera", m_cameras[Cam.FRONT.idx].photonCamera.isConnected());
        SmartDashboard.putBoolean("aprilTagVision/backCamera", m_cameras[Cam.BACK.idx].photonCamera.isConnected());
    }

    public void updateSimulation(SwerveDrive swerve) {
        if (SwerveDriveTelemetry.isSimulation && swerve.getSimulationDriveTrainPose().isPresent()) {
            //  In the maple-sim, odometry is simulated using encoder values, accounting for
            //  factors like skidding and drifting.
            //  As a result, the odometry may not always be 100% accurate.
            //  However, the vision system should be able to provide a reasonably accurate
            //  pose estimation, even when odometry is incorrect.
            //  (This is why teams implement vision system to correct odometry.)
            //  Therefore, we must ensure that the actual robot pose is provided in the
            //  simulator when updating the vision simulation during the simulation.       
            m_visionSim.update(swerve.getSimulationDriveTrainPose().get());
        }
    }

    public void updateOdometry(SwerveDrive swerve) {

        // Cannot do anything if there is no field layout
        if (m_aprilTagFieldLayout == null)
            return;

        try {
            if (PLOT_VISIBLE_TAGS) {
                plotVisibleTags(swerve.field);
            }

            // Since we want to go through the images twice, we need to fetch the results and save them
            // getAllUnreadResults() forgets the results once called
            for (Camera c : m_cameras) {
                c.pipeResults = c.photonCamera.getAllUnreadResults();
            }

            // Do MultiTag
            addVisionMeasurements(swerve, true);

            // Do SingleTag
            addVisionMeasurements(swerve, false);
        } catch (Exception e) {
            DriverStation.reportError("Error updating odometry from AprilTags " + e.getLocalizedMessage(), false);
        }
    }

    public void addVisionMeasurements(SwerveDrive swerve, boolean useMultiTag) {
        Pose2d robotPose = swerve.getPose();

        for (Camera c : m_cameras) {
            try {
                c.poseEstimator.setReferencePose(robotPose);

                for (PhotonPipelineResult pipeRes : c.pipeResults) {
                    
                    // Important: PhotonPoseEstimator will not run if the result is the same time as the last call
                    // So we can't actually run through the list twice
                    // But, we can test for Multitag before calling the poseEstimator
                    if (useMultiTag == pipeRes.multitagResult.isPresent()) {
                        Optional<EstimatedRobotPose> estPose = c.poseEstimator.update(pipeRes);
                        if (estPose.isPresent()) {
                            // Update the main poseEstimator with the vision result
                            // Make sure to use the timestamp of this result
                            swerve.addVisionMeasurement(estPose.get().estimatedPose.toPose2d(), pipeRes.getTimestampSeconds(), estimateStdDev(robotPose, pipeRes.targets));
                        }
                    }
                }
            } catch (Exception e) {
                // bad! log this and keep going
                DriverStation.reportError("Exception running PhotonPoseEstimator", e.getStackTrace());
            }
        }
    }

    // // FROM HERE TO THE BOTTOM, ALL ARE NOT USED
    // List<Pose3d> getOptions(Camera camera, Optional<EstimatedRobotPose> estimate) {
    //     List<Pose3d> options = new ArrayList<Pose3d>();
    //     if (estimate.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
    //         // if multitag is used, add robot pose to options
    //         options.add(estimate.get().estimatedPose);
    //     } else {
    //         // if only one tag is visible, add all possible poses to options
    //         options = getAmbiguousPoses(camera.photonCamera.getLatestResult(), camera.robotToCam);
    //     }

    //     return options;
    // }

    // void plotAndUpdate(Camera camera, Optional<EstimatedRobotPose> estimate, SwerveDrive swerve) {
    //     Pose2d pose = estimate.get().estimatedPose.toPose2d();
    //     swerve.addVisionMeasurement(pose, camera.photonCamera.getLatestResult().getTimestampSeconds());

    //     if (PLOT_POSE_SOLUTIONS) {
    //         plotVisionPose(swerve.field, pose);
    //     }
    //     if (PLOT_ALTERNATE_POSES) {
    //         // *** Yes, this is repeated code, and maybe that is bad.
    //         // But this will save some cycles if this PLOT option is turned off.
    //         if (estimate.get().strategy != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
    //             plotAlternateSolutions(swerve.field,
    //                     List.of(getAmbiguousPoses(camera.photonCamera.getLatestResult(), camera.robotToCam)));
    //         } else
    //             swerve.field.getObject("visionAltPoses").setPose(pose);
    //     }
    // }

    // get the tag ID closest to horizontal center of camera
    // we might want to use this to do fine adjustments on field element locations
    public int getCentralTagId() {
        // make sure camera connected
        if (!m_cameras[Cam.FRONT.idx].photonCamera.isConnected())
            return -1;

        var targetResult = m_cameras[Cam.FRONT.idx].photonCamera.getLatestResult();
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

    // Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations based
    // on number of tags, estimation strategy, and distance from the tags.
    private Matrix<N3, N1> estimateStdDev(Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {

        // Pose present. Start running Heuristic
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an
        // average-distance metric
        for (PhotonTrackedTarget tgt : targets) {
            var tagPose = m_aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;

            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }

        // Should not happen, but protect against divide by zero
        if (numTags == 0)
            return INFINITE_STDDEV;

        avgDist /= numTags;

        // Single tags further away than 4 meter (~13 ft) are useless
        if (numTags == 1 && avgDist > 4.0) 
            return INFINITE_STDDEV;

        // Starting estimate = multitag or not
        Matrix<N3, N1> estStdDev = numTags == 1 ? SINGLE_TAG_BASE_STDDEV : MULTI_TAG_BASE_STDDEV;

        // Increase std devs based on (average) distance
        // This is taken from YAGSL vision example.
        // TODO figure out why
        estStdDev = estStdDev.times(1.0 + avgDist * avgDist / 30.0);

        return estStdDev;
    }

    // Private routines for calculating the odometry info

    // // create a strategy based off closestToReferencePoseStrategy that returns all
    // // possible robot positions
    // private static ArrayList<Pose3d> getAmbiguousPoses(PhotonPipelineResult result, Transform3d robotToCamera) {
    //     ArrayList<Pose3d> ambigiousPoses = new ArrayList<>();
    //     for (PhotonTrackedTarget target : result.targets) {
    //         int targetFiducialId = target.getFiducialId();

    //         // Don't report errors for non-fiducial targets. This could also be resolved by
    //         // adding -1 to
    //         // the initial HashSet.
    //         if (targetFiducialId == -1)
    //             continue;

    //         Optional<Pose3d> targetPosition = m_aprilTagFieldLayout.getTagPose(target.getFiducialId());

    //         if (targetPosition.isEmpty())
    //             continue;

    //         // add all possible robot positions to the array that is returned
    //         ambigiousPoses.add(
    //                 targetPosition.get()
    //                         .transformBy(target.getBestCameraToTarget().inverse())
    //                         .transformBy(robotToCamera.inverse()));
    //         ambigiousPoses.add(
    //                 targetPosition.get()
    //                         .transformBy(target.getAlternateCameraToTarget().inverse())
    //                         .transformBy(robotToCamera.inverse()));
    //     }

    //     return ambigiousPoses;
    // }

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
        prop.setAvgLatencyMs(20.0);
        prop.setLatencyStdDevMs(5.0);

        // Note: NetworkTables does not update the timestamp of an entry if the value does not change.
        // The timestamp is used by PVLib to know if there is a new frame, so in a simulation
        // with no uncertainty, it thinks that it is not detecting a tag if the robot is static.
        // So, always have a little bit of uncertainty.
        prop.setCalibError(0.1, 0.03);

        for (Camera c : m_cameras) {
            PhotonCameraSim camSim = new PhotonCameraSim(c.photonCamera, prop);
            camSim.setMaxSightRange(Units.feetToMeters(20.0));
            m_visionSim.addCamera(camSim, c.robotToCam);
        }

        m_visionSim.addAprilTags(m_aprilTagFieldLayout);
    }

    // --- Routines to plot the vision solutions on a Field2d ---------

    // private void plotVisionPoses(Field2d field, List<Pose2d> poses) {
    //     if (field == null)
    //         return;
    //     if (poses == null)
    //         field.getObject("visionPoses").setPoses();
    //     else
    //         field.getObject("visionPoses").setPoses(poses);
    // }

    // private void plotVisionPose(Field2d field, Pose2d pose) {
    //     if (field == null)
    //         return;
    //     field.getObject("visionPoses").setPose(pose);
    // }

    private void plotVisibleTags(Field2d field) {
        if (field == null)
            return;

        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        for (Camera cam : m_cameras) {
            if (!cam.photonCamera.isConnected()) continue;

            for (PhotonTrackedTarget target : cam.photonCamera.getLatestResult().getTargets()) {
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

    // private void plotAlternateSolutions(Field2d field, List<List<Pose3d>> allPoses) {
    //     if (field == null)
    //         return;

    //     ArrayList<Pose2d> both = new ArrayList<>();
    //     for (List<Pose3d> pl : allPoses) {
    //         for (Pose3d p : pl)
    //             both.add(p.toPose2d());
    //     }

    //     field.getObject("visionAltPoses").setPoses(both);
    // }
}