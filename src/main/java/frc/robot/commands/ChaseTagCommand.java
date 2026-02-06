package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ChaseTagCommand extends Command {

        private final VisionSubsystem vision;
        private final SwerveSubsystem drive;
        private final double targetDistanceMeters;
        private final int[] tagIds;

        private final PIDController yawController;
        private final PIDController distanceController;

        private boolean hasTarget = false;

        public ChaseTagCommand(
                        VisionSubsystem vision,
                        SwerveSubsystem drive,
                        int[] tagIds,
                        double targetDistanceMeters) {

                this.vision = vision;
                this.drive = drive;
                this.tagIds = tagIds;
                this.targetDistanceMeters = targetDistanceMeters;

                // ---------------- YAW (ANGLE) PID ----------------
                yawController = new PIDController(
                                VisionConstants.SHOOTER_YAW_KP,
                                0.0,
                                VisionConstants.SHOOTER_YAW_KD);
                yawController.enableContinuousInput(-Math.PI, Math.PI);
                yawController.setTolerance(
                                VisionConstants.SHOOTER_YAW_TOLERANCE_RAD);

                // ---------------- DISTANCE PID ----------------
                distanceController = new PIDController(
                                VisionConstants.SHOOTER_DISTANCE_KP,
                                0.0,
                                VisionConstants.SHOOTER_DISTANCE_KD);
                distanceController.setTolerance(
                                VisionConstants.SHOOTER_DISTANCE_TOLERANCE_M);

                addRequirements(drive);
        }

        @Override
        public void initialize() {
                yawController.reset();
                distanceController.reset();
                hasTarget = false;
        }

        @Override
        public void execute() {

                var yawOpt = vision.getTargetYawRad(tagIds);
                var distOpt = vision.getDistanceToTagMeters(tagIds);

                if (yawOpt.isEmpty() || distOpt.isEmpty()) {
                        drive.drive(new ChassisSpeeds());
                        hasTarget = false;
                        SmartDashboard.putBoolean("ChaseTag/HasTarget", false);
                        return;
                }

                hasTarget = true;

                // Rear-facing camera: rotate yaw by 180°
                double yawRad = MathUtil.angleModulus(yawOpt.get() + Math.PI);
                double distanceM = distOpt.get();

                // ---------------- PID CALCULATIONS ----------------

                // Forward/back (invert because camera faces rear)
                double rawXSpeed = -distanceController.calculate(
                                distanceM,
                                targetDistanceMeters);

                // Lateral correction (camera centering)
                double rawYSpeed = yawController.calculate(
                                yawRad,
                                0.0);

                // Rotation to face tag
                double rawOmega = -yawController.calculate(
                                yawRad,
                                0.0);

                // ---------------- CLAMP OUTPUTS ----------------

                double xSpeed = MathUtil.clamp(
                                rawXSpeed,
                                -VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED,
                                VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED);

                double ySpeed = MathUtil.clamp(
                                rawYSpeed,
                                -VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED,
                                VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED);

                double omega = MathUtil.clamp(
                                rawOmega,
                                -VisionConstants.SHOOTER_MAX_ANGULAR_SPEED,
                                VisionConstants.SHOOTER_MAX_ANGULAR_SPEED);

                // ---------------- ANGLE TELEMETRY ----------------

                double currentAngleRad = drive.getPose().getRotation().getRadians();

                double targetAngleRad = MathUtil.angleModulus(currentAngleRad + yawRad);

                double angleErrorRad = MathUtil.angleModulus(targetAngleRad - currentAngleRad);

                // ---------------- TELEMETRY ----------------

                SmartDashboard.putBoolean("ChaseTag/HasTarget", true);

                // Vision yaw
                SmartDashboard.putNumber("ChaseTag/YawRad", yawRad);
                SmartDashboard.putNumber("ChaseTag/YawDeg", Math.toDegrees(yawRad));
                SmartDashboard.putNumber("ChaseTag/YawPIDOutput", rawYSpeed);
                SmartDashboard.putBoolean("ChaseTag/YawAtSetpoint", yawController.atSetpoint());

                // Robot angles
                SmartDashboard.putNumber("ChaseTag/CurrentAngleRad", currentAngleRad);
                SmartDashboard.putNumber("ChaseTag/CurrentAngleDeg",
                                Math.toDegrees(currentAngleRad));

                SmartDashboard.putNumber("ChaseTag/TargetAngleRad", targetAngleRad);
                SmartDashboard.putNumber("ChaseTag/TargetAngleDeg",
                                Math.toDegrees(targetAngleRad));

                SmartDashboard.putNumber("ChaseTag/AngleErrorRad", angleErrorRad);
                SmartDashboard.putNumber("ChaseTag/AngleErrorDeg",
                                Math.toDegrees(angleErrorRad));

                // Distance
                SmartDashboard.putNumber("ChaseTag/DistanceMeters", distanceM);
                SmartDashboard.putNumber(
                                "ChaseTag/DistanceErrorMeters",
                                targetDistanceMeters - distanceM);
                SmartDashboard.putNumber("ChaseTag/DistancePIDOutput", rawXSpeed);

                // Final commands
                SmartDashboard.putNumber("ChaseTag/XSpeedCmd", xSpeed);
                SmartDashboard.putNumber("ChaseTag/YSpeedCmd", ySpeed);
                SmartDashboard.putNumber("ChaseTag/OmegaCmd", omega);

                // ---------------- DRIVE ----------------
                drive.drive(new ChassisSpeeds(
                                xSpeed,
                                ySpeed,
                                omega));
        }

        @Override
        public void end(boolean interrupted) {
                drive.drive(new ChassisSpeeds());
        }

        @Override
        public boolean isFinished() {
                return hasTarget
                                && yawController.atSetpoint()
                                && distanceController.atSetpoint();
        }
}
