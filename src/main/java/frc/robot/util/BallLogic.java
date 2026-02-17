package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BallLogic {

    /**
     * Reorders points so the robot drives to the nearest ball, then the next
     * nearest to THAT ball.
     */
    public static ArrayList<Pose2d> rearrangePoints(Pose2d startpoint, List<Pose2d> points, int indexLimit) {
        ArrayList<Pose2d> remainingPoints = new ArrayList<>(points);
        ArrayList<Pose2d> optimizedPoseList = new ArrayList<>();

        optimizedPoseList.add(startpoint);

        // Limit the search to either the number of balls seen or your limit
        int actualLimit = Math.min(indexLimit, remainingPoints.size());

        for (int i = 0; i < actualLimit; i++) {
            Pose2d currentPos = optimizedPoseList.get(optimizedPoseList.size() - 1);
            Pair<Pose2d, Integer> closest = findClosestPoint(remainingPoints, currentPos);

            if (closest.getFirst() != null) {
                // Set the rotation so the robot faces the ball as it drives to it
                Rotation2d angleToBall = getRotation2dToPose(currentPos, closest.getFirst());
                Pose2d targetedBall = new Pose2d(closest.getFirst().getTranslation(), angleToBall);

                optimizedPoseList.add(targetedBall);
                remainingPoints.remove((int) closest.getSecond());
            }
        }
        return optimizedPoseList;
    }

    public static Pair<Pose2d, Integer> findClosestPoint(List<Pose2d> points, Pose2d currentPoint) {
        double shortestDistance = Double.POSITIVE_INFINITY;
        Pose2d closestPose = null;
        int index = -1;

        for (int i = 0; i < points.size(); i++) {
            double dist = currentPoint.getTranslation().getDistance(points.get(i).getTranslation());
            if (dist < shortestDistance) {
                shortestDistance = dist;
                closestPose = points.get(i);
                index = i;
            }
        }
        return new Pair<>(closestPose, index);
    }

    public static Rotation2d getRotation2dToPose(Pose2d startPose, Pose2d targetPose) {
        double xdis = targetPose.getX() - startPose.getX();
        double ydis = targetPose.getY() - startPose.getY();
        return new Rotation2d(Math.atan2(ydis, xdis));
    }
}