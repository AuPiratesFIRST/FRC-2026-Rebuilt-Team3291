package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ChaseTowerTagCommand extends Command {

    // SLOWER constraints so the robot doesn't tip over when the elevator is raised
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 1.5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 1.5);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(2.0, 2.0);

    private final VisionSubsystem vision;
    private final SwerveSubsystem swerve;
    private final int[] tagIds;
    
    private final double targetDistanceMeters;
    private final double lateralOffsetMeters;

    private final ProfiledPIDController xController = new ProfiledPIDController(2.5, 0, 0.05, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(2.5, 0, 0.05, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2.0, 0, 0.05, OMEGA_CONSTRAINTS);

    private PhotonTrackedTarget lastTarget;
    private Pose2d fieldGoalPose;

    public ChaseTowerTagCommand(
            VisionSubsystem vision,
            SwerveSubsystem swerve,
            int[] tagIds,
            double targetDistanceMeters,
            double lateralOffsetMeters) {

        this.vision = vision;
        this.swerve = swerve;
        this.tagIds = tagIds;
        this.targetDistanceMeters = targetDistanceMeters;
        this.lateralOffsetMeters = lateralOffsetMeters;

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        omegaController.setTolerance(Units.degreesToRadians(2));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        fieldGoalPose = null;
        var robotPose = swerve.getPose();

        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        omegaController.reset(robotPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        var robotPose2d = swerve.getPose();
        var robotPose3d = new Pose3d(robotPose2d);

        // 1. Grab results from BOTH cameras
        var frontRes = vision.getFrontCameraResult();
        var rearRes = vision.getLatestResult(); // Your shooter cam

        boolean usingFrontCam = false;
        PhotonTrackedTarget bestTarget = null;

        // 2. Prioritize Front Cam, fallback to Shooter (Rear) Cam
        if (frontRes.hasTargets() && getValidTarget(frontRes) != null) {
            bestTarget = getValidTarget(frontRes);
            usingFrontCam = true;
        } else if (rearRes.hasTargets() && getValidTarget(rearRes) != null) {
            bestTarget = getValidTarget(rearRes);
            usingFrontCam = false;
        }

        if (bestTarget != null) {
            lastTarget = bestTarget;

            // 3. Set proper Camera Transform
            Transform3d robotToCam = usingFrontCam ? 
                VisionConstants.ROBOT_TO_FRONT_CAMERA : 
                VisionConstants.ROBOT_TO_SHOOTER_CAMERA;

            // 4. Handle 180-degree flip logic for the rear camera
            // If using the front camera, rotation is PI (facing the tag).
            // If using the rear camera, rotation is 0 (facing away from the tag).
            double rotOffset = usingFrontCam ? Math.PI : 0.0;
            
            // If the robot is spun 180 degrees to use the rear camera, your left-mounted climber 
            // is now on the opposite side of the center line relative to the tower.
            // We invert the lateral offset so the robot shifts to put the climber on the same target rung!
            double appliedLateralOffset = usingFrontCam ? lateralOffsetMeters : -lateralOffsetMeters;

            Transform3d tagToGoal = new Transform3d(
                    new Translation3d(targetDistanceMeters, appliedLateralOffset, 0.0),
                    new Rotation3d(0.0, 0.0, rotOffset));

            // 5. Calculate Field Goal Pose
            var cameraPose = robotPose3d.transformBy(robotToCam);
            var targetPose = cameraPose.transformBy(bestTarget.getBestCameraToTarget());
            fieldGoalPose = targetPose.transformBy(tagToGoal).toPose2d();

            xController.setGoal(fieldGoalPose.getX());
            yController.setGoal(fieldGoalPose.getY());
            omegaController.setGoal(fieldGoalPose.getRotation().getRadians());
            
            SmartDashboard.putBoolean("ChaseTag/UsingFrontCam", usingFrontCam);
        }

        // 6. Drive the Robot
        if (lastTarget == null || fieldGoalPose == null) {
            swerve.drive(new ChassisSpeeds());
        } else {
            var xSpeed = xController.calculate(robotPose2d.getX());
            var ySpeed = yController.calculate(robotPose2d.getY());
            var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());

            if (xController.atGoal()) xSpeed = 0;
            if (yController.atGoal()) ySpeed = 0;
            if (omegaController.atGoal()) omegaSpeed = 0;

            swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));

            SmartDashboard.putNumber("ChaseTag/GoalX", fieldGoalPose.getX());
            SmartDashboard.putNumber("ChaseTag/GoalY", fieldGoalPose.getY());
            SmartDashboard.putNumber("ChaseTag/GoalDeg", fieldGoalPose.getRotation().getDegrees());
        }
    }

    private PhotonTrackedTarget getValidTarget(org.photonvision.targeting.PhotonPipelineResult result) {
        return result.getTargets().stream()
                .filter(t -> isIdValid(t.getFiducialId()))
                .filter(t -> t.getPoseAmbiguity() <= 0.2 && t.getPoseAmbiguity() != -1)
                .findFirst().orElse(null);
    }

    private boolean isIdValid(int id) {
        for (int validId : tagIds) {
            if (id == validId) return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return fieldGoalPose != null && xController.atGoal() && yController.atGoal() && omegaController.atGoal();
    }
}
