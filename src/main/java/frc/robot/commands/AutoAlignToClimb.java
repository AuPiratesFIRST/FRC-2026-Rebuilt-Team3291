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

    public enum Side { LEFT, RIGHT }

    public AutoAlignToClimb(VisionSubsystem vision, SwerveSubsystem swerve, ElevatorSubsystem elevator, Side side) {
        
        // 1. DETERMINE OFFSETS (Based on Page 27 Tower Dimensions)
        // Rungs are 16.125" (0.41m) from center.
        double lateralOffset = (side == Side.LEFT) ? 0.41 : -0.41;

        // 2. DEFINE THE NEIGHBORHOOD POSE
        // Blue Alliance Tower Center is roughly at X=0, Y=4.0. 
        // We approach from X=1.2m away.
        Pose2d blueTowerApproach = new Pose2d(new Translation2d(1.2, 4.0 + lateralOffset), Rotation2d.fromDegrees(180));
        
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

            // PHASE 2: PRECISION VISION DOCK ON THAT SIDE
            new ParallelCommandGroup(
                elevator.setHeight(Meters.of(0.3)),
                // Pass the specific tower tags (IDs 15, 16, 31, 32 per Page 34)
                new ChaseFrontTagCommand(vision, swerve, new int[]{15, 16, 31, 32}, 0.6, lateralOffset)
                    .withTimeout(1.5)
            )
        );
    }
}
