package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class ObjectDetection extends SubsystemBase {
    private final DoubleArraySubscriber xSub, ySub;
    private final IntegerSubscriber numSub;
    private final Field2d m_field;
    private final SwerveSubsystem m_swerve;

    private List<Pose2d> ballPosesFieldRelative = new ArrayList<>();

    public ObjectDetection(Field2d field, SwerveSubsystem swerve) {
        this.m_field = field;
        this.m_swerve = swerve;
        NetworkTable table = NetworkTableInstance.getDefault().getTable("fuelCV");

        numSub = table.getIntegerTopic("number_of_fuel").subscribe(0);
        xSub = table.getDoubleArrayTopic("ball_position_x").subscribe(new double[] {});
        ySub = table.getDoubleArrayTopic("ball_position_y").subscribe(new double[] {});
    }

    @Override
    public void periodic() {
        double[] xMeters = xSub.get();
        double[] yMeters = ySub.get();
        int count = (int) numSub.get();

        Pose2d robotPose = m_swerve.getPose();
        ballPosesFieldRelative.clear();

        for (int i = 0; i < count && i < xMeters.length; i++) {
            // 1. Add Camera Offset (The camera is at +0.25m from center)
            double robotRelativeX = xMeters[i] + VisionConstants.kCameraXOffset;
            double robotRelativeY = yMeters[i] + VisionConstants.kCameraYOffset;

            // 2. Rotate relative coordinates by Robot Heading to get Field coordinates
            Translation2d ballTranslation = new Translation2d(robotRelativeX, robotRelativeY)
                    .rotateBy(robotPose.getRotation());

            Pose2d fieldPose = new Pose2d(
                    robotPose.getX() + ballTranslation.getX(),
                    robotPose.getY() + ballTranslation.getY(),
                    new Rotation2d());

            ballPosesFieldRelative.add(fieldPose);
        }

        // Display on Map
        m_field.getObject("detected_balls").setPoses(ballPosesFieldRelative);
        Logger.recordOutput("Vision/BallsOnField", ballPosesFieldRelative.toArray(new Pose2d[0]));
    }

    public List<Pose2d> getBallPosesFieldRelative() {
        return ballPosesFieldRelative;
    }
}