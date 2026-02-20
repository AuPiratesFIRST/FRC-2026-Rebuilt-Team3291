package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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

public class ChaseTagCommand extends Command {

        private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3.0, 2.0);
        private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3.0, 2.0);
        private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(4.0,
                        4.0);

        private final VisionSubsystem vision;
        private final SwerveSubsystem swerve;
        private final int[] tagIds;
        private final Transform3d tagToGoal;

        private final ProfiledPIDController xController = new ProfiledPIDController(3.0, 0, 0.05, X_CONSTRAINTS);
        private final ProfiledPIDController yController = new ProfiledPIDController(3.0, 0, 0.05, Y_CONSTRAINTS);
        private final ProfiledPIDController omegaController = new ProfiledPIDController(2.0, 0, 0.05,
                        OMEGA_CONSTRAINTS);

        private PhotonTrackedTarget lastTarget;
        private Pose2d fieldGoalPose;

        public ChaseTagCommand(
                        VisionSubsystem vision,
                        SwerveSubsystem swerve,
                        int[] tagIds,
                        double targetDistanceMeters) {
                this.vision = vision;
                this.swerve = swerve;
                this.tagIds = tagIds;

                // FIX: Change Rotation3d from 0.0 to Math.PI
                // In this field-relative logic, Math.PI ensures the robot's internal "0"
                // (front)
                // faces away from the tag, meaning the REAR faces the tag.
                this.tagToGoal = new Transform3d(
                                new Translation3d(targetDistanceMeters, 0.0, 0.0),
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

                var photonRes = vision.getLatestResult();

                if (photonRes.hasTargets()) {
                        var targetOpt = photonRes.getTargets().stream()
                                        .filter(t -> isIdValid(t.getFiducialId()))
                                        .filter(t -> t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                                        .findFirst();

                        if (targetOpt.isPresent()) {
                                var target = targetOpt.get();
                                lastTarget = target;

                                // 1. Calculate camera position in field space
                                var cameraPose = robotPose3d.transformBy(VisionConstants.ROBOT_TO_SHOOTER_CAMERA);
                                // 2. Calculate target position in field space
                                var targetPose = cameraPose.transformBy(target.getBestCameraToTarget());
                                // 3. Calculate goal position (where robot center should be) in field space
                                fieldGoalPose = targetPose.transformBy(tagToGoal).toPose2d();

                                xController.setGoal(fieldGoalPose.getX());
                                yController.setGoal(fieldGoalPose.getY());
                                omegaController.setGoal(fieldGoalPose.getRotation().getRadians());
                        }
                }

                if (lastTarget == null || fieldGoalPose == null) {
                        swerve.drive(new ChassisSpeeds());
                } else {
                        // Calculate field-relative speeds
                        var xSpeed = xController.calculate(robotPose2d.getX());
                        var ySpeed = yController.calculate(robotPose2d.getY());
                        var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());

                        if (xController.atGoal())
                                xSpeed = 0;
                        if (yController.atGoal())
                                ySpeed = 0;
                        if (omegaController.atGoal())
                                omegaSpeed = 0;

                        // Convert Field-Relative to Robot-Relative
                        swerve.drive(
                                        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed,
                                                        robotPose2d.getRotation()));

                        SmartDashboard.putNumber("ChaseTag/GoalX", fieldGoalPose.getX());
                        SmartDashboard.putNumber("ChaseTag/GoalY", fieldGoalPose.getY());
                        SmartDashboard.putNumber("ChaseTag/GoalDeg", fieldGoalPose.getRotation().getDegrees());
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
                return fieldGoalPose != null && xController.atGoal() && yController.atGoal()
                                && omegaController.atGoal();
        }
}