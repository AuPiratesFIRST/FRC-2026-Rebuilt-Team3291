package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonUtils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.BallLogic;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.ObjectDetection;

public class PathfindThroughBalls extends Command {
    private final SwerveSubsystem mSwerve;
    private final ObjectDetection mDetection;

    private PathPlannerPath path;
    private Command followPath;

    public PathfindThroughBalls(SwerveSubsystem swerve, ObjectDetection detection) {
        this.mSwerve = swerve;
        this.mDetection = detection;
        addRequirements(mSwerve, mDetection);
    }

    @Override
    public void initialize() {
        // FIX: Calling the correct method name from your ObjectDetection subsystem
        List<Pose2d> detectedBalls = mDetection.getBallPosesFieldRelative();

        if (detectedBalls.size() > 0) {
            // 1. Organize points into an efficient path (limit to closest 3 balls)
            ArrayList<Pose2d> properPoints = BallLogic.rearrangePoints(
                    mSwerve.getPose(),
                    new ArrayList<>(detectedBalls),
                    3);

            // 2. Create rotation targets so the intake faces the balls during travel
            ArrayList<RotationTarget> rotations = new ArrayList<>();
            for (int i = 1; i < properPoints.size(); i++) {
                rotations.add(new RotationTarget(i / (double) properPoints.size(), properPoints.get(i).getRotation()));
            }

            // 3. Construct the Path using your constraints
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(properPoints);

            path = new PathPlannerPath(
                    waypoints,
                    rotations,
                    new ArrayList<>(), new ArrayList<>(), new ArrayList<>(),
                    new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
                    null,
                    new GoalEndState(0, properPoints.get(properPoints.size() - 1).getRotation()),
                    false);

            path.preventFlipping = true;

            // 4. Generate the path-following command and initialize it
            followPath = AutoBuilder.followPath(path);
            followPath.initialize();

            System.out.println("PathfindThroughBalls: Path generated with " + properPoints.size() + " points.");
        } else {
            System.out.println("PathfindThroughBalls: No balls detected to pathfind to.");
        }
    }

    @Override
    public void execute() {
        if (followPath != null) {
            followPath.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (followPath != null) {
            followPath.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        // Finish when PathPlanner says the path is complete
        return followPath != null && followPath.isFinished();
    }
}