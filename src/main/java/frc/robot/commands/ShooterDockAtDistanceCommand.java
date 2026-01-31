package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShooterDockAtDistanceCommand extends Command {

    private final VisionSubsystem vision;
    private final SwerveSubsystem drive;
    private final double targetDistanceMeters;

    private final PIDController yawController;
    private final PIDController distanceController;

    private boolean hasTarget = false;

    public ShooterDockAtDistanceCommand(
            VisionSubsystem vision,
            SwerveSubsystem drive,
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

        var yawOpt = vision.getShooterTargetYawRad(hubTags);
        var distOpt = vision.getDistanceToTagMeters(hubTags);

        if (yawOpt.isEmpty() || distOpt.isEmpty()) {
            drive.drive(new ChassisSpeeds());
            hasTarget = false;
            return;
        }

        hasTarget = true;

        double yawRad = yawOpt.get();
        double distanceM = distOpt.get();

        // Rotate to face hub center
        double omega = yawController.calculate(yawRad, 0.0);

        // Drive to desired shooting distance
        double xSpeed = distanceController.calculate(
                distanceM,
                targetDistanceMeters);

        omega = MathUtil.clamp(
                omega,
                -VisionConstants.SHOOTER_MAX_ANGULAR_SPEED,
                VisionConstants.SHOOTER_MAX_ANGULAR_SPEED);

        xSpeed = MathUtil.clamp(
                xSpeed,
                -VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED,
                VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED);

        drive.drive(new ChassisSpeeds(
                xSpeed,
                0.0,
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

    private int[] getAllianceHubTags() {
        return DriverStation.getAlliance()
                .filter(a -> a == DriverStation.Alliance.Red)
                .map(a -> VisionConstants.RED_HUB_TAGS)
                .orElse(VisionConstants.BLUE_HUB_TAGS);
    }
}
