package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

/**
 * AimShooterFromVision - Automatically aims shooter based on vision distance.
 * 
 * This command continuously reads the distance to AprilTags from vision and
 * calculates the optimal shooter RPM and hood angle for that distance.
 * 
 * How it works:
 * 1. Query vision for distance to hub AprilTags
 * 2. Use ShooterAimCalculator to convert distance → RPM + hood angle
 * 3. Apply those settings to shooter and hood subsystems
 * 4. If vision lost, use fallback settings (safe default shot)
 * 
 * When to use:
 * - Hold operator trigger while aiming at hub
 * - Command runs continuously, updating aim as distance changes
 * - Releases control when button released (shooter stops)
 * 
 * Requirements:
 * - Requires shooter and hood (prevents other commands from interfering)
 * - Does NOT require vision (vision is read-only, no exclusive access needed)
 */
public class AimShooterFromVision extends Command {

    // Subsystems we control
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    // Subsystem we read from (no exclusive access needed)
    private final VisionSubsystem vision;

    /**
     * Creates a new AimShooterFromVision command.
     * 
     * @param shooter Flywheel subsystem to set RPM
     * @param hood Angle adjustment subsystem
     * @param vision Vision subsystem to read distance from
     */
    public AimShooterFromVision(
            ShooterSubsystem shooter,
            HoodSubsystem hood,
            VisionSubsystem vision) {

        this.shooter = shooter;
        this.hood = hood;
        this.vision = vision;

        // Reserve shooter and hood so no other commands can use them
        // Vision is NOT added because we only read from it
        addRequirements(shooter, hood);
    }

    /**
     * Called repeatedly while command is scheduled (every 20ms).
     * Reads vision distance and updates shooter aim.
     */
    @Override
    public void execute() {

        // Determine which AprilTags are valid targets based on alliance
        // Blue alliance aims at blue hub tags, red aims at red hub tags
        int[] validTags;

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (alliance == Alliance.Red) {
            validTags = VisionConstants.RED_HUB_TAGS;  // Tag IDs for red hub
        } else {
            validTags = VisionConstants.BLUE_HUB_TAGS;  // Tag IDs for blue hub
        }

        // Ask vision subsystem: "How far away are we from any of these tags?"
        // Returns Optional - could be empty if no tags visible
        Optional<Double> distance = vision.getDistanceToTagMeters(validTags);

        ShooterAimCalculator.ShooterSolution solution;

        // ---------------- FALLBACK LOGIC ----------------
        // If vision can't see tags OR distance is invalid, use safe fallback
        if (distance.isEmpty()) {
            // No vision target found - use conservative default shot
            solution = ShooterAimCalculator.fallback();
        } else {
            // Vision found target - calculate optimal aim for this distance
            solution = ShooterAimCalculator.solve(distance.get());
        }

        // Double-check solution is valid (in case distance was out of range)
        if (!solution.valid()) {
            solution = ShooterAimCalculator.fallback();
        }

        // ---------------- APPLY OUTPUTS ----------------
        // Send calculated settings to shooter and hood
        hood.applyAngle(solution.hoodAngle());    // Set hood angle (degrees)
        shooter.applyRPM(solution.rpm());         // Set flywheel speed (RPM)
    }

    @Override
    public void end(boolean interrupted) {
        // Let default commands take over
        shooter.stop().schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
