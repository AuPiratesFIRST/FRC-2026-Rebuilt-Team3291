package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class ObjectDetection extends SubsystemBase {
    private final DoubleArraySubscriber xSub, ySub;
    private final IntegerSubscriber numSub;

    // Target Piles (The "Strategic" data from Python)
    private final DoubleSubscriber targetXSub, targetYSub;
    private final BooleanSubscriber hasTargetSub;

    private final Field2d m_field;
    private final SwerveSubsystem m_swerve;

    private List<Pose2d> ballPosesFieldRelative = new ArrayList<>();

    public ObjectDetection(Field2d field, SwerveSubsystem swerve) {
        this.m_field = field;
        this.m_swerve = swerve;
        NetworkTable table = NetworkTableInstance.getDefault().getTable("fuelCV");

        // Existing array data
        numSub = table.getIntegerTopic("number_of_fuel").subscribe(0);
        xSub = table.getDoubleArrayTopic("ball_position_x").subscribe(new double[] {});
        ySub = table.getDoubleArrayTopic("ball_position_y").subscribe(new double[] {});

        // NEW: Specific target data for the HUD
        hasTargetSub = table.getBooleanTopic("has_target").subscribe(false);
        targetXSub = table.getDoubleTopic("target_x").subscribe(0.0);
        targetYSub = table.getDoubleTopic("target_y").subscribe(0.0);
    }

    @Override
    public void periodic() {
        double[] xMeters = xSub.get();
        double[] yMeters = ySub.get();
        int count = (int) numSub.get();

        Pose2d robotPose = m_swerve.getPose();
        ballPosesFieldRelative.clear();

        // 1. Process all detected balls for the map
        for (int i = 0; i < count && i < xMeters.length; i++) {
            double robotRelativeX = xMeters[i] + VisionConstants.kCameraXOffset;
            double robotRelativeY = yMeters[i] + VisionConstants.kCameraYOffset;

            Translation2d ballTranslation = new Translation2d(robotRelativeX, robotRelativeY)
                    .rotateBy(robotPose.getRotation());

            Pose2d fieldPose = new Pose2d(
                    robotPose.getX() + ballTranslation.getX(),
                    robotPose.getY() + ballTranslation.getY(),
                    new Rotation2d());

            ballPosesFieldRelative.add(fieldPose);
        }

        // 2. Update the "All Balls" view (Yellow dots)
        m_field.getObject("detected_balls").setPoses(ballPosesFieldRelative);

        // 3. Update the "Target HUD" (Highlight the best pile)
        if (hasTargetSub.get()) {
            // Calculate where the target pile is in Field Coordinates
            Translation2d pileRelTranslation = new Translation2d(
                    targetXSub.get() + VisionConstants.kCameraXOffset,
                    targetYSub.get() + VisionConstants.kCameraYOffset).rotateBy(robotPose.getRotation());

            Pose2d targetPilePose = new Pose2d(
                    robotPose.getX() + pileRelTranslation.getX(),
                    robotPose.getY() + pileRelTranslation.getY(),
                    new Rotation2d());

            // Draw the target as a "Target" icon on the map
            m_field.getObject("CurrentTarget").setPose(targetPilePose);
            SmartDashboard.putString("Vision/HUD_Status", "LOCKED ON PILE");
        } else {
            // Hide the target if nothing is found
            m_field.getObject("CurrentTarget").setPoses(new ArrayList<>());
            SmartDashboard.putString("Vision/HUD_Status", "SEARCHING...");
        }

        // 4. AdvantageKit Logging
        Logger.recordOutput("Vision/BallsOnField", ballPosesFieldRelative.toArray(new Pose2d[0]));
    }

    public List<Pose2d> getBallPosesFieldRelative() {
        return ballPosesFieldRelative;
    }
}