package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.ObjectDetection;
import java.util.List;
import java.util.function.DoubleSupplier;

public class ManualChaseBall extends Command {
    private final SwerveSubsystem mSwerve;
    private final ObjectDetection mDetection;

    // Joystick inputs
    private final DoubleSupplier m_translationX;
    private final DoubleSupplier m_translationY;

    // We only need a PID for Rotation (Theta)
    private final PIDController thetaController = new PIDController(4.0, 0, 0);

    public ManualChaseBall(
            SwerveSubsystem swerve,
            ObjectDetection detection,
            DoubleSupplier translationX,
            DoubleSupplier translationY) {

        this.mSwerve = swerve;
        this.mDetection = detection;
        this.m_translationX = translationX;
        this.m_translationY = translationY;

        addRequirements(mSwerve);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // Tighter tolerance for the intake alignment
        thetaController.setTolerance(Units.degreesToRadians(2));
    }

    @Override
    public void execute() {
        // 1. Get Human Input (Driving)
        double xSpeed = m_translationX.getAsDouble() * mSwerve.getSwerveDrive().getMaximumChassisVelocity();
        double ySpeed = m_translationY.getAsDouble() * mSwerve.getSwerveDrive().getMaximumChassisVelocity();

        // 2. Get Vision Data (Ball Location)
        List<Pose2d> balls = mDetection.getBallPosesFieldRelative();
        double omegaSpeed = 0;

        if (!balls.isEmpty()) {
            Pose2d targetBall = balls.get(0);
            Pose2d currentPose = mSwerve.getPose();

            // Calculate angle from Robot center to Ball
            Translation2d robotToBall = targetBall.getTranslation().minus(currentPose.getTranslation());
            Rotation2d angleToBall = robotToBall.getAngle();

            // 3. Calculate Automatic Rotation
            omegaSpeed = thetaController.calculate(
                    currentPose.getRotation().getRadians(),
                    angleToBall.getRadians());

            // Limit turning speed so it's controllable but snappy
            omegaSpeed = MathUtil.clamp(omegaSpeed, -4.0, 4.0);
        }

        // 4. Drive: Human controls X/Y, Computer controls Omega
        mSwerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, omegaSpeed, mSwerve.getPose().getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        mSwerve.drive(new ChassisSpeeds(0, 0, 0));
    }
}