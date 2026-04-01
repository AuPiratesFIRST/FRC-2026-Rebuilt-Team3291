package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import static edu.wpi.first.units.Units.Meters;
import java.util.Set;

public class AutoAlignToClimb extends SequentialCommandGroup {

    public enum Side {
        LEFT, RIGHT
    }

    public AutoAlignToClimb(VisionSubsystem vision, SwerveSubsystem swerve, ElevatorSubsystem elevator, Side side) {

        // ===========================================================================
        // 1. DETERMINE OFFSETS
        // ===========================================================================

        // --- Y-Axis (Lateral) Measurements ---
        double uprightDistance = 0.85;
        double robotHalfWidth = 0.4235; // TODO: Adjust to your robot's actual half-width in meters!

        // --- X-Axis (Depth) Measurements for PathPlanner ---
        // Tune these independently!
        // Decreasing the number drives the robot CLOSER to the tower.
        // Increasing the number stops the robot FURTHER AWAY.
        double leftFieldApproachX = 1.153; // Was 1.353. Subtracted 0.2m because you said it stopped short.
        double rightFieldApproachX = 1.053; // Was 1.053. You said this side was perfectly tuned.

        // --- X-Axis (Depth) Measurements for Vision Docking ---
        // Tune these independently for the final vision approach!
        double leftVisionDistance = 0.95; // (Previously 0.8 + 0.15)
        double rightVisionDistance = 0.65; // (Previously 0.8 - 0.15)

        // Variables that will be dynamically set based on the side chosen
        double fieldLateralOffset;
        double visionLateralOffset;
        double fieldApproachX;
        double visionTargetDistance;
        double targetRotationDegrees;

        if (side == Side.LEFT) {
            // LEFT SIDE: Rotate 180 (faces tower, uses front cam)
            targetRotationDegrees = 180.0;

            // Apply independent LEFT offsets
            fieldApproachX = leftFieldApproachX;
            visionTargetDistance = leftVisionDistance;

            // Lateral (Y) Offsets
            fieldLateralOffset = uprightDistance - robotHalfWidth;
            visionLateralOffset = fieldLateralOffset;

        } else {
            // RIGHT SIDE: Rotate 0 (faces away from tower, uses rear/shooter cam)
            targetRotationDegrees = 0.0;

            // Apply independent RIGHT offsets
            fieldApproachX = rightFieldApproachX;
            visionTargetDistance = rightVisionDistance;

            // Lateral (Y) Offsets
            fieldLateralOffset = -uprightDistance - robotHalfWidth;
            visionLateralOffset = uprightDistance + robotHalfWidth; // Inverse for the camera flip
        }

        // ===========================================================================
        // 2. DEFINE THE NEIGHBORHOOD POSE
        // ===========================================================================
        Pose2d blueTowerApproach = new Pose2d(
                new Translation2d(fieldApproachX, 3.964 + fieldLateralOffset),
                Rotation2d.fromDegrees(targetRotationDegrees));

        PathConstraints constraints = new PathConstraints(2.8, 2, Math.PI, 2 * Math.PI);

        addCommands(
                // PHASE 1: PATHFIND TO THE SPECIFIC SIDE
                new DeferredCommand(() -> {
                    Pose2d target = blueTowerApproach;
                    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                        target = FlippingUtil.flipFieldPose(blueTowerApproach);
                    }
                    return AutoBuilder.pathfindToPose(target, constraints, 0.0);
                }, Set.of(swerve)),

                // PHASE 2: PRECISION VISION DOCK ON THAT SIDE (Using Unified Command)
                new ParallelCommandGroup(
                        elevator.goToHeight(Meters.of(0.3))
                // new ChaseTowerTagCommand(
                // vision,
                // swerve,
                // new int[] { 15, 16, 31, 32 }, // Tower Tags
                // 0.8, // Target distance in meters
                // visionLateralOffset).withTimeout(2.5) // Slightly longer timeout to allow
                // backing up
                // safely
                ));
    }
}
