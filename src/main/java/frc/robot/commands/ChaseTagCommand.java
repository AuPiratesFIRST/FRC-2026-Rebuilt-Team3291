package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

        private final PIDController yawController;
        private final PIDController strafeController;
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
                yawController.setTolerance(VisionConstants.SHOOTER_YAW_TOLERANCE_RAD);

                // ---------------- STRAFE (LATERAL) PID ----------------
                strafeController = new PIDController(
                                VisionConstants.SHOOTER_Strafe_KP,
                                0.0,
                                VisionConstants.SHOOTER_Strafe_KD);
                strafeController.setTolerance(VisionConstants.SHOOTER_STRAFE_TOLERANCE_M);

                // ---------------- DISTANCE PID ----------------
                distanceController = new PIDController(
                                VisionConstants.SHOOTER_DISTANCE_KP,
                                0.0,
                                VisionConstants.SHOOTER_DISTANCE_KD);
                distanceController.setTolerance(VisionConstants.SHOOTER_DISTANCE_TOLERANCE_M);

                addRequirements(drive);
        }

        @Override
        public void initialize() {
                yawController.reset();
                strafeController.reset();
                distanceController.reset();
                hasTarget = false;
        }

        @Override
        public void execute() {
                // --- LIVE TUNING ---
                yawController.setPID(VisionConstants.SHOOTER_YAW_KP, 0, VisionConstants.SHOOTER_YAW_KD);
                strafeController.setPID(VisionConstants.SHOOTER_Strafe_KP, 0, VisionConstants.SHOOTER_Strafe_KD);
                distanceController.setPID(VisionConstants.SHOOTER_DISTANCE_KP, 0, VisionConstants.SHOOTER_DISTANCE_KD);

                var yawOpt = vision.getTargetYawRad(tagIds);
                var distOpt = vision.getDistanceToTagMeters(tagIds);
                var lateralOpt = vision.getTargetLateralOffsetMeters(tagIds); // NEW: 3D Y-Offset

                if (yawOpt.isEmpty() || distOpt.isEmpty() || lateralOpt.isEmpty()) {
                        drive.drive(new ChassisSpeeds());
                        hasTarget = false;
                        SmartDashboard.putBoolean("ChaseTag/HasTarget", false);
                        return;
                }
                hasTarget = true;

                // --- DATA ---
                double yawError = yawOpt.get(); // Radians (for rotation)
                double lateralError = lateralOpt.get(); // Meters (for strafing)
                double distanceM = distOpt.get(); // Meters (for forward/back)

                // ---------------- PID CALCULATIONS ----------------

                // 1. FORWARD/BACK (X): To move toward a tag behind the robot, drive Negative X.
                double rawXSpeed = -distanceController.calculate(distanceM, targetDistanceMeters);

                // 2. STRAFE (Y):
                // In Camera Space: +Y is Left.
                // Because the camera is REAR-FACING: +Y (Camera Left) = Robot Right.
                // To center a tag that is to the Robot's Right, move Robot Right (-Y velocity).
                double rawYSpeed = -strafeController.calculate(lateralError, 0.0);

                // 3. ROTATION (Omega):
                // To rotate the rear bumper toward a tag that is to the Camera's Right (-Yaw),
                // the robot must rotate Clockwise (Negative Omega).
                double rawOmega = yawController.calculate(yawError, 0.0);

                // ---------------- CLAMP OUTPUTS ----------------
                double xSpeed = MathUtil.clamp(rawXSpeed, -VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED,
                                VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED);
                double ySpeed = MathUtil.clamp(rawYSpeed, -VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED,
                                VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED);
                double omega = MathUtil.clamp(rawOmega, -VisionConstants.SHOOTER_MAX_ANGULAR_SPEED,
                                VisionConstants.SHOOTER_MAX_ANGULAR_SPEED);

                // ---------------- TELEMETRY & LOGGING ----------------

                double currentAngleRad = drive.getPose().getRotation().getRadians();
                // Target angle is current angle + the yaw offset to the tag
                double targetAngleRad = MathUtil.angleModulus(currentAngleRad + yawError);
                double angleErrorRad = MathUtil.angleModulus(targetAngleRad - currentAngleRad);

                // AdvangeKit Logging for Graphs
                Logger.recordOutput("ChaseTag/Distance/Actual", distanceM);
                Logger.recordOutput("ChaseTag/Distance/Target", targetDistanceMeters);
                Logger.recordOutput("ChaseTag/Angle/ErrorDeg", Math.toDegrees(angleErrorRad));

                // ---------------- ALL REQUESTED TELEMETRY ----------------

                SmartDashboard.putBoolean("ChaseTag/HasTarget", true);

                // Vision yaw
                SmartDashboard.putNumber("ChaseTag/YawRad", yawError);
                SmartDashboard.putNumber("ChaseTag/YawDeg", Math.toDegrees(yawError));
                SmartDashboard.putNumber("ChaseTag/YawPIDOutput", rawYSpeed);
                SmartDashboard.putBoolean("ChaseTag/YawAtSetpoint", yawController.atSetpoint());

                // Robot angles
                SmartDashboard.putNumber("ChaseTag/CurrentAngleRad", currentAngleRad);
                SmartDashboard.putNumber("ChaseTag/CurrentAngleDeg", Math.toDegrees(currentAngleRad));

                SmartDashboard.putNumber("ChaseTag/TargetAngleRad", targetAngleRad);
                SmartDashboard.putNumber("ChaseTag/TargetAngleDeg", Math.toDegrees(targetAngleRad));

                SmartDashboard.putNumber("ChaseTag/AngleErrorRad", angleErrorRad);
                SmartDashboard.putNumber("ChaseTag/AngleErrorDeg", Math.toDegrees(angleErrorRad));

                // Distance
                SmartDashboard.putNumber("ChaseTag/DistanceMeters", distanceM);
                SmartDashboard.putNumber("ChaseTag/DistanceErrorMeters", targetDistanceMeters - distanceM);
                SmartDashboard.putNumber("ChaseTag/DistancePIDOutput", rawXSpeed);

                // Final commands
                SmartDashboard.putNumber("ChaseTag/XSpeedCmd", xSpeed);
                SmartDashboard.putNumber("ChaseTag/YSpeedCmd", ySpeed);
                SmartDashboard.putNumber("ChaseTag/OmegaCmd", omega);

                // ---------------- DRIVE ----------------
                drive.drive(new ChassisSpeeds(xSpeed, ySpeed, omega));
        }

        @Override
        public void end(boolean interrupted) {
                drive.drive(new ChassisSpeeds());
        }

        @Override
        public boolean isFinished() {
                return hasTarget && yawController.atSetpoint() && distanceController.atSetpoint();
        }
}