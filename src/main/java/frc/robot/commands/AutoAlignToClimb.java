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

        // 1. DETERMINE OFFSETS
        // Uprights are ~16.125" (0.41m) from center, but using your 0.81m custom offset
        double uprightDistance = 0.85;
        double robotHalfWidth = 0.3935; // TODO: Adjust to your robot's actual half-width in meters!

        double fieldLateralOffset;
        double visionLateralOffset;
        double targetRotationDegrees;

        if (side == Side.LEFT) {
            // LEFT SIDE: Rotate 180 (faces tower, uses front cam)
            targetRotationDegrees = 180.0;

            // PathPlanner Field Offset
            // Target the +Y upright, shift robot to the right (-Y direction relative to
            // upright)
            // so the left side of the robot hooks it.
            fieldLateralOffset = uprightDistance - robotHalfWidth;

            // Vision Offset (Front camera applies this directly)
            visionLateralOffset = fieldLateralOffset;
        } else {
            // RIGHT SIDE: Rotate 0 (faces away from tower, uses rear/shooter cam)
            targetRotationDegrees = 0.0;

            // PathPlanner Field Offset
            // Target the -Y upright. Since robot is reversed, its physical left side is
            // pointing towards +Y.
            // So we shift the robot further into the -Y direction.
            fieldLateralOffset = -uprightDistance - robotHalfWidth;

            // Vision Offset
            // IMPORTANT: Because our ChaseTowerTagCommand automatically flips the lateral
            // offset when using the rear camera, we need to pass the INVERSE here so that
            // when it flips it internally, it evaluates to the correct field offset.
            // Inverse of (-0.81 - halfWidth) is (+0.81 + halfWidth)
            visionLateralOffset = uprightDistance + robotHalfWidth;
        }

        // 2. DEFINE THE NEIGHBORHOOD POSE
        Pose2d blueTowerApproach = new Pose2d(
                new Translation2d(1.193, 3.964 + fieldLateralOffset),
                Rotation2d.fromDegrees(targetRotationDegrees));

        PathConstraints constraints = new PathConstraints(2.0, 1.5, Math.PI, 2 * Math.PI);

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
                        elevator.setHeight(Meters.of(0.3)),
                        new ChaseTowerTagCommand(
                                vision,
                                swerve,
                                new int[] { 15, 16, 31, 32 }, // Tower Tags
                                0.8, // Target distance in meters
                                visionLateralOffset).withTimeout(2.5) // Slightly longer timeout to allow backing up
                                                                      // safely
                ));
    }
}
