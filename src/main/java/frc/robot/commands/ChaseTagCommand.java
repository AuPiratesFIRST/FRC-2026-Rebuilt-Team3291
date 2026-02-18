package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

public class ChaseTagCommand extends Command {
        private final VisionSubsystem vision;
        private final SwerveSubsystem drive;
        private final double targetDistanceMeters;
        private final int[] tagIds;

        private final PIDController xController;
        private final PIDController yController;
        private final PIDController rotController;

        private boolean hasTarget = false;

        public ChaseTagCommand(VisionSubsystem vision, SwerveSubsystem drive, int[] tagIds,
                        double targetDistanceMeters) {
                this.vision = vision;
                this.drive = drive;
                this.tagIds = tagIds;
                this.targetDistanceMeters = targetDistanceMeters;

                // X: Forward/Back Gains
                xController = new PIDController(VisionConstants.SHOOTER_DISTANCE_KP, 0,
                                VisionConstants.SHOOTER_DISTANCE_KD);

                // Y: Strafe Gains (1.5)
                yController = new PIDController(VisionConstants.SHOOTER_Strafe_KP, 0,
                                VisionConstants.SHOOTER_Strafe_KD);

                // Rot: Rotation Gains (0.3)
                rotController = new PIDController(VisionConstants.SHOOTER_YAW_KP, 0, VisionConstants.SHOOTER_YAW_KD);
                rotController.enableContinuousInput(-Math.PI, Math.PI);

                addRequirements(drive);
        }

        @Override
        public void initialize() {
                xController.reset();
                yController.reset();
                rotController.reset();
                hasTarget = false;
        }

        @Override
        public void execute() {
                // --- LIVE TUNING ---
                xController.setPID(VisionConstants.SHOOTER_DISTANCE_KP, 0, VisionConstants.SHOOTER_DISTANCE_KD);
                yController.setPID(VisionConstants.SHOOTER_Strafe_KP, 0, VisionConstants.SHOOTER_Strafe_KD);
                rotController.setPID(VisionConstants.SHOOTER_YAW_KP, 0, VisionConstants.SHOOTER_YAW_KD);

                // --- GET POSE IN TARGET SPACE ---
                var botPoseTargetSpaceOpt = vision.getRobotPoseInTargetSpace(tagIds);

                if (botPoseTargetSpaceOpt.isEmpty()) {
                        drive.drive(new ChassisSpeeds());
                        hasTarget = false;
                        SmartDashboard.putBoolean("ChaseTag/HasTarget", false);
                        return;
                }

                hasTarget = true;
                Pose2d botPose = botPoseTargetSpaceOpt.get();

                // --- TARGET SPACE LOGIC ---
                // Goal X = distance we want to be from the tag
                // Goal Y = 0 (perfectly centered)
                // Goal Rot = 0 (perfectly square/parallel to tag)

                // calculate(Actual, Setpoint)
                double rawX = xController.calculate(botPose.getX(), targetDistanceMeters);
                double rawY = yController.calculate(botPose.getY(), 0.0);
                double rawOmega = rotController.calculate(botPose.getRotation().getRadians(), 0.0);

                // --- REAR CAMERA INVERSION ---
                // Because the camera is on the back, the robot needs to move
                // in the -X direction to close the distance.
                double finalX = -rawX;
                double finalY = -rawY;
                double finalOmega = -rawOmega;

                // --- DRIVE ---
                drive.drive(new ChassisSpeeds(
                                MathUtil.clamp(finalX, -VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED,
                                                VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED),
                                MathUtil.clamp(finalY, -VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED,
                                                VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED),
                                MathUtil.clamp(finalOmega, -VisionConstants.SHOOTER_MAX_ANGULAR_SPEED,
                                                VisionConstants.SHOOTER_MAX_ANGULAR_SPEED)));

                // --- ALL TELEMETRY (Kept for you) ---
                SmartDashboard.putBoolean("ChaseTag/HasTarget", true);
                SmartDashboard.putNumber("ChaseTag/DistanceM", botPose.getX());
                SmartDashboard.putNumber("ChaseTag/LateralM", botPose.getY());
                SmartDashboard.putNumber("ChaseTag/RotationDeg", botPose.getRotation().getDegrees());
                SmartDashboard.putNumber("ChaseTag/XSpeedCmd", finalX);
                SmartDashboard.putNumber("ChaseTag/YSpeedCmd", finalY);
                SmartDashboard.putNumber("ChaseTag/OmegaCmd", finalOmega);

                Logger.recordOutput("ChaseTag/BotPoseTargetSpace", botPose);
        }

        @Override
        public void end(boolean interrupted) {
                drive.drive(new ChassisSpeeds());
        }

        @Override
        public boolean isFinished() {
                return hasTarget && xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
        }
}