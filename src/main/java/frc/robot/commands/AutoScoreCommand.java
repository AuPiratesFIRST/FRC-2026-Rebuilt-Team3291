package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.TankDrive.Drive;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Pathfinds to a scoring approach pose, then uses vision
 * to precisely dock at shooting distance.
 *
 * Tank-drive safe.
 */
public class AutoScoreCommand extends SequentialCommandGroup {

        public AutoScoreCommand(Drive drive, VisionSubsystem vision) {

                /* --------- APPROACH POSE (VISION DOES FINAL ALIGN) --------- */
                Pose2d scoringApproachPose = new Pose2d(
                                2.409,
                                3.962,
                                Rotation2d.fromDegrees(180));

                /* --------- PATHFINDING CONSTRAINTS --------- */
                PathConstraints constraints = new PathConstraints(
                                2.5, // max velocity (m/s)
                                2.0, // max acceleration (m/s^2)
                                Math.PI, // max angular velocity (rad/s)
                                2 * Math.PI // max angular acceleration (rad/s^2)
                );

                /* --------- PATHFIND COMMAND (CORRECT OVERLOAD) --------- */
                Command pathfindToScore = AutoBuilder.pathfindToPose(
                                scoringApproachPose,
                                constraints,
                                0.0 // goal end velocity (m/s)
                );

                /* --------- FINAL VISION ALIGN --------- */
                Command visionAlign = new ShooterDockAtDistanceCommand(
                                vision,
                                drive,
                                VisionConstants.SHOOTING_DISTANCE_METERS);

                addCommands(pathfindToScore, visionAlign);
        }
}
