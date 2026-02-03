package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.TankDrive.Drive;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShooterDockAtDistanceCommand extends Command {

        private final VisionSubsystem vision;
        private final Drive drive;
        private final double targetDistanceMeters;

        private final PIDController yawController;
        private final PIDController distanceController;

        private boolean hasTarget = false;

        public ShooterDockAtDistanceCommand(
                        VisionSubsystem vision,
                        Drive drive,
                        double targetDistanceMeters) {

                this.vision = vision;
                this.drive = drive;
                this.targetDistanceMeters = targetDistanceMeters;

                yawController = new PIDController(
                                VisionConstants.SHOOTER_YAW_KP,
                                0.0,
                                VisionConstants.SHOOTER_YAW_KD);
                yawController.enableContinuousInput(-Math.PI, Math.PI);
                yawController.setTolerance(
                                VisionConstants.SHOOTER_YAW_TOLERANCE_RAD);

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
                int[] hubTags = getAllianceHubTags();

                var yawOpt = vision.getTargetYawRad(hubTags);
                var distOpt = vision.getDistanceToTagMeters(hubTags);

                // Do NOT clear hasTarget on vision dropout
                if (yawOpt.isEmpty() || distOpt.isEmpty()) {
                        drive.runClosedLoop(new ChassisSpeeds());
                        return;
                }

                hasTarget = true;

                // ✅ CORRECT PID CONVENTION
                // measurement, setpoint
                double omega = yawController.calculate(yawOpt.get(), 0.0);

                // ✅ CORRECT DRIVE DIRECTION
                double xSpeed = distanceController.calculate(
                                distOpt.get(),
                                targetDistanceMeters);

                omega = MathUtil.clamp(
                                omega,
                                -VisionConstants.SHOOTER_MAX_ANGULAR_SPEED,
                                VisionConstants.SHOOTER_MAX_ANGULAR_SPEED);

                xSpeed = MathUtil.clamp(
                                xSpeed,
                                -VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED,
                                VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED);

                // Robot-relative (correct for tank)
                drive.runClosedLoop(new ChassisSpeeds(xSpeed, 0.0, omega));
        }

        @Override
        public void end(boolean interrupted) {
                drive.runClosedLoop(new ChassisSpeeds());
        }

        @Override
        public boolean isFinished() {
                return hasTarget
                                && yawController.atSetpoint()
                                && distanceController.atSetpoint();
        }

        private int[] getAllianceHubTags() {
                return DriverStation.getAlliance()
                                .filter(a -> a == DriverStation.Alliance.Red)
                                .map(a -> VisionConstants.RED_HUB_TAGS)
                                .orElse(VisionConstants.BLUE_HUB_TAGS);
        }
}
