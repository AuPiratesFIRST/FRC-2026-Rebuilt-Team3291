package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.TankDrive.Drive;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Set;

/**
 * AutoScoreCommand - Fully autonomous scoring sequence.
 * 
 * This command combines pathfinding and vision alignment to automatically
 * navigate to the scoring position and align for shooting.
 * 
 * Sequence:
 * 1. Pathfind to a scoring position near the hub (using PathPlanner)
 * 2. Use vision to fine-tune distance from hub (get to exact shooting distance)
 * 
 * The scoring position is defined in BLUE alliance coordinates and automatically
 * flipped for RED alliance using PathPlanner's field mirroring.
 * 
 * Alliance-safe: Works correctly regardless of which alliance we're on!
 * Tank-drive safe: Uses DeferredCommand to ensure alliance check happens when command runs
 */
public class AutoScoreCommand extends SequentialCommandGroup {

        public AutoScoreCommand(Drive drive, VisionSubsystem vision) {

                Pose2d bluePose = new Pose2d(
                                2.544,
                                5.787,
                                Rotation2d.fromDegrees(36.39));

                PathConstraints constraints = new PathConstraints(
                                2.5,
                                2.0,
                                Math.PI,
                                2 * Math.PI);

                /* --------- DEFERRED PATHFIND (ALLIANCE SAFE) --------- */
                Command pathfindToScore = new DeferredCommand(
                                () -> {
                                        Pose2d targetPose = bluePose;
                                        DriverStation.reportWarning(
                                                        "AUTO TARGET POSE: " + targetPose, false);

                                        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                                                targetPose = FlippingUtil.flipFieldPose(bluePose);
                                        }

                                        return AutoBuilder.pathfindToPose(
                                                        targetPose,
                                                        constraints,
                                                        0.0);
                                },

                                Set.of(drive)

                );

                Command visionAlign = new ShooterDockAtDistanceCommand(
                                vision,
                                drive,
                                VisionConstants.SHOOTING_DISTANCE_METERS);

                addCommands(pathfindToScore, visionAlign);
        }
}
