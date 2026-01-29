package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.*;
import org.photonvision.simulation.*;
import org.photonvision.targeting.*;

public class VisionSubsystem extends SubsystemBase {

    // ---------- CAMERAS ----------
    private final PhotonCamera frontCamera;
    private final PhotonCamera shooterCamera;

    // ---------- POSE ----------
    private final PhotonPoseEstimator poseEstimator;
    private final StructPublisher<Pose2d> visionPosePub;
    private final AprilTagFieldLayout fieldLayout;

    // ---------- SIM ----------
    private VisionSystemSim visionSim;
    private PhotonCameraSim frontCamSim;
    private PhotonCameraSim shooterCamSim;

    public VisionSubsystem() {

        frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
        shooterCamera = new PhotonCamera(SHOOTER_CAMERA_NAME);

        fieldLayout = AprilTagFieldLayout.loadField(
                AprilTagFields.k2026RebuiltWelded);

        poseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                ROBOT_TO_FRONT_CAMERA);

        visionPosePub = NetworkTableInstance.getDefault()
                .getStructTopic("Vision/EstimatedPose", Pose2d.struct)
                .publish();

        if (RobotBase.isSimulation()) {
            setupSimulation();
        }
    }

    // ---------------- SIM ----------------
    private void setupSimulation() {

        visionSim = new VisionSystemSim("Vision");

        visionSim.addAprilTags(fieldLayout);

        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        props.setFPS(30);
        props.setAvgLatencyMs(35);

        frontCamSim = new PhotonCameraSim(frontCamera, props);
        shooterCamSim = new PhotonCameraSim(shooterCamera, props);

        visionSim.addCamera(frontCamSim, ROBOT_TO_FRONT_CAMERA);
        visionSim.addCamera(shooterCamSim, ROBOT_TO_SHOOTER_CAMERA);
    }

    public void updateSimPose(Pose2d robotPose) {
        if (visionSim != null) {
            visionSim.update(robotPose);
        }
    }

    // ---------------- POSE ----------------
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var result = frontCamera.getLatestResult();
        if (!result.hasTargets())
            return Optional.empty();
        return poseEstimator.update(result);
    }

    // ---------------- DISTANCE (SHOOTER CAMERA) ----------------
    public Optional<Double> getDistanceToTagMeters(int[] validTags) {

        PhotonPipelineResult result = shooterCamera.getLatestResult();

        if (!result.hasTargets())
            return Optional.empty();

        for (PhotonTrackedTarget target : result.getTargets()) {
            for (int id : validTags) {
                if (target.getFiducialId() == id) {

                    Transform3d camToTarget = target.getBestCameraToTarget();

                    return Optional.of(
                            camToTarget.getTranslation().getNorm());
                }
            }
        }
        return Optional.empty();
    }

    @Override
    public void periodic() {
        getEstimatedGlobalPose()
                .ifPresent(p -> visionPosePub.set(
                        p.estimatedPose.toPose2d()));
    }
}
