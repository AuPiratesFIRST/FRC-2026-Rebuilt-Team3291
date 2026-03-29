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
        // Uprights are 32.25" apart, which means they are ~16.125" (0.41m) from the center.
        double uprightOffset = (side == Side.LEFT) ? 0.41 : -0.41;

        // Because your climber is on the LEFT side of the robot, you need to shift the 
        // center of the robot to the RIGHT so the left edge hooks the upright.
        double robotHalfWidth = 0.35; // TODO: Adjust to your robot's actual half-width in meters!
        double lateralOffset = uprightOffset - robotHalfWidth;

        // 2. DEFINE THE NEIGHBORHOOD POSE
        // We are BACKING UP to the Blue Tower (X=0). 
        // This means the robot's front faces AWAY from the tower -> Rotation is 0 degrees.
        Pose2d blueTowerApproach = new Pose2d(new Translation2d(1.2, 4.0 + lateralOffset), Rotation2d.fromDegrees(0));
        
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
                new SmartChaseTagCommand(
                    vision, 
                    swerve, 
                    new int[]{15, 16, 31, 32}, // Tower Tags
                    0.8, // Target distance in meters
                    lateralOffset
                ).withTimeout(2.5) // Slightly longer timeout to allow backing up safely
            )
        );
    }
}
