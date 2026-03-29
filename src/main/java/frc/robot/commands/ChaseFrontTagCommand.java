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

public class ChaseFrontTagCommand extends Command {

    // SLOWER constraints so the robot doesn't tip over when the elevator is raised
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 1.5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 1.5);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(2.0, 2.0);

    private final VisionSubsystem vision;
    private final SwerveSubsystem swerve;
    private final int[] tagIds;
    private final Transform3d tagToGoal;

    private final ProfiledPIDController xController = new ProfiledPIDController(2.5, 0, 0.05, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(2.5, 0, 0.05, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2.0, 0, 0.05, OMEGA_CONSTRAINTS);

    private PhotonTrackedTarget lastTarget;
    private Pose2d fieldGoalPose;

    public ChaseFrontTagCommand(
            VisionSubsystem vision,
            SwerveSubsystem swerve,
            int[] tagIds,
            double targetDistanceMeters,
            double lateralOffsetMeters) {

        this.vision = vision;
        this.swerve = swerve;
        this.tagIds = tagIds;

        /// Use lateralOffsetMeters to shift the robot left or right of the tag
        // Positive Y is left, Negative Y is right
        this.tagToGoal = new Transform3d(
                new Translation3d(targetDistanceMeters, lateralOffsetMeters, 0.0),
                new Rotation3d(0.0, 0.0, Math.PI));

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

        // USE THE FRONT CAMERA!
        var photonRes = vision.getFrontCameraResult();

        if (photonRes.hasTargets()) {
            var targetOpt = photonRes.getTargets().stream()
                    .filter(t -> isIdValid(t.getFiducialId()))
                    .filter(t -> t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                    .findFirst();

            if (targetOpt.isPresent()) {
                var target = targetOpt.get();
                lastTarget = target;

                // USE THE FRONT CAMERA OFFSET!
                var cameraPose = robotPose3d.transformBy(VisionConstants.ROBOT_TO_FRONT_CAMERA);
                var targetPose = cameraPose.transformBy(target.getBestCameraToTarget());
                fieldGoalPose = targetPose.transformBy(tagToGoal).toPose2d();

                xController.setGoal(fieldGoalPose.getX());
                yController.setGoal(fieldGoalPose.getY());
                omegaController.setGoal(fieldGoalPose.getRotation().getRadians());
            }
        }

        if (lastTarget == null || fieldGoalPose == null) {
            swerve.drive(new ChassisSpeeds());
        } else {
            var xSpeed = xController.calculate(robotPose2d.getX());
            var ySpeed = yController.calculate(robotPose2d.getY());
            var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());

            if (xController.atGoal())
                xSpeed = 0;
            if (yController.atGoal())
                ySpeed = 0;
            if (omegaController.atGoal())
                omegaSpeed = 0;

            swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
        }
    }

    private boolean isIdValid(int id) {
        for (int validId : tagIds) {
            if (id == validId)
                return true;
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
