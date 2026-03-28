package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import static edu.wpi.first.units.Units.Meters;

public class AutoAlignToClimb extends SequentialCommandGroup {

    public AutoAlignToClimb(
            VisionSubsystem vision,
            SwerveSubsystem swerve,
            ElevatorSubsystem elevator,
            int[] towerTagIds) {

        // The exact distance to stop so your hooks are right above the RUNG
        double finalDockingDistance = 0.6; // <-- TUNE THIS ON YOUR ROBOT

        // The safe distance to start raising the elevator (Prevents tipping)
        double safeElevatorDistance = 2.2; // <-- TUNE THIS ON YOUR ROBOT

        addCommands(
                // PHASE 1: Fast approach with the elevator DOWN (keeps Center of Gravity low)
                // It runs until the robot is within the safeElevatorDistance.
                new ChaseTagCommand(vision, swerve, towerTagIds, safeElevatorDistance)
                        .until(() -> isCloseEnough(vision, towerTagIds, safeElevatorDistance)),

                // PHASE 2: Run two commands at the exact same time!
                // Raise the elevator to the high rung AND creep to the final distance
                // simultaneously
                new ParallelCommandGroup(
                        elevator.setHeight(Meters.of(1.5)), // <-- TUNE THIS to your climb height
                        new ChaseTagCommand(vision, swerve, towerTagIds, finalDockingDistance)
                                .until(() -> isCloseEnough(vision, towerTagIds, finalDockingDistance + 0.05))));
    }

    // Helper method to check distance to the tag using the front camera
    private boolean isCloseEnough(VisionSubsystem vision, int[] tagIds, double threshold) {
        var res = vision.getFrontCameraResult();
        if (res.hasTargets()) {
            var targetOpt = res.getTargets().stream()
                    .filter(t -> {
                        for (int id : tagIds)
                            if (id == t.getFiducialId())
                                return true;
                        return false;
                    }).findFirst();

            if (targetOpt.isPresent()) {
                // Get the straight-line distance to the tag
                double dist = targetOpt.get().getBestCameraToTarget().getTranslation().getNorm();
                return dist <= threshold;
            }
        }
        return false;
    }
}