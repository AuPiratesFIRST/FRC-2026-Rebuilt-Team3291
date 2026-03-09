package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.ObjectDetection;
import java.util.List;

public class ChaseBall extends Command {
    private final SwerveSubsystem mSwerve;
    private final ObjectDetection mDetection;

    // PID Controllers
    private final PIDController xController = new PIDController(2.0, 0, 0);
    private final PIDController yController = new PIDController(2.0, 0, 0);
    private final PIDController thetaController = new PIDController(3.0, 0, 0);

    public ChaseBall(SwerveSubsystem swerve, ObjectDetection detection) {
        this.mSwerve = swerve;
        this.mDetection = detection;
        addRequirements(mSwerve);

        // Wrap rotation PID to handle -PI to +PI correctly
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        List<Pose2d> balls = mDetection.getBallPosesFieldRelative();

        if (balls.isEmpty()) {
            mSwerve.drive(new ChassisSpeeds(0, 0, 0));
            return;
        }

        Pose2d targetBall = balls.get(0);
        Pose2d currentPose = mSwerve.getPose();

        // 1. Calculate direction to ball
        Translation2d robotToBall = targetBall.getTranslation().minus(currentPose.getTranslation());
        Rotation2d angleToBall = robotToBall.getAngle();

        // 2. Define the target point: 0.5 meters in front of the ball
        // We look at the vector to the ball and stop 0.5m short
        double approachDistance = Math.max(0, robotToBall.getNorm() - 0.5);
        Translation2d targetPoint = currentPose.getTranslation().plus(
                new Translation2d(approachDistance, angleToBall));

        // 3. Calculate speeds
        double xSpeed = xController.calculate(currentPose.getX(), targetPoint.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPoint.getY());

        // We want the robot to face the ball
        double omegaSpeed = thetaController.calculate(
                currentPose.getRotation().getRadians(),
                angleToBall.getRadians());

        // 4. Drive
        mSwerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, omegaSpeed, currentPose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        mSwerve.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        // Stop if we are within a small margin of the 0.5m approach point
        List<Pose2d> balls = mDetection.getBallPosesFieldRelative();
        if (balls.isEmpty())
            return true;

        double dist = balls.get(0).getTranslation().getDistance(mSwerve.getPose().getTranslation());
        return dist < 0.6; // If we are within 0.6m of the ball, we are at our 0.5m target
    }
}