package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

        // Yaw PID → ROTATION (omega)
        yawController = new PIDController(
                VisionConstants.SHOOTER_YAW_KP,
                0.0,
                VisionConstants.SHOOTER_YAW_KD);
        yawController.enableContinuousInput(-Math.PI, Math.PI);
        yawController.setTolerance(
                VisionConstants.SHOOTER_YAW_TOLERANCE_RAD);

        // Distance PID → FORWARD/BACK
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
            return;
        }

        hasTarget = true;

        double yawRad = yawOpt.get();
        double distanceM = distOpt.get();

        // Forward/back: get to target distance
        double xSpeed = distanceController.calculate(
                targetDistanceMeters,
                distanceM);

        // Strafe: keep tag centered in camera
        double ySpeed = yawController.calculate(
                yawRad,
                0.0);

        // Rotate robot to face tag (slower than strafe)
        double omega = yawController.calculate(
                yawRad,
                0.0);

        xSpeed = MathUtil.clamp(
                xSpeed,
                -VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED,
                VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED);

        ySpeed = MathUtil.clamp(
                ySpeed,
                -VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED,
                VisionConstants.SHOOTER_MAX_TRANSLATION_SPEED);

        omega = MathUtil.clamp(
                omega,
                -VisionConstants.SHOOTER_MAX_ANGULAR_SPEED,
                VisionConstants.SHOOTER_MAX_ANGULAR_SPEED);

        // ROBOT-RELATIVE chase
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
