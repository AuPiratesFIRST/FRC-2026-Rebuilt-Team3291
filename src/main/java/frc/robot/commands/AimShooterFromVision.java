package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;

import java.util.Optional;

public class AimShooterFromVision extends Command {

    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final VisionSubsystem vision;

    // 1. Median Filter: Ignores random "spikes" (e.g., if a frame says 100m by
    // accident)
    // Size 5 means it takes the middle value of the last 5 frames.
    private final MedianFilter outlierFilter = new MedianFilter(5);

    // 2. Moving Average: Smooths out the tiny jitters (e.g., 2.45m to 2.47m)
    // Size 10 means it averages the last 10 frames (~0.2 seconds of data).
    private final LinearFilter smoothFilter = LinearFilter.movingAverage(10);

    private double filteredDistance = 0;

    public AimShooterFromVision(
            ShooterSubsystem shooter,
            HoodSubsystem hood,
            VisionSubsystem vision) {
        this.shooter = shooter;
        this.hood = hood;
        this.vision = vision;
        addRequirements(shooter, hood);
    }

    @Override
    public void execute() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        int[] validTags = (alliance == Alliance.Red) ? VisionConstants.RED_HUB_TAGS : VisionConstants.BLUE_HUB_TAGS;

        Optional<Double> rawDistance = vision.getDistanceToTagMeters(validTags);

        ShooterAimCalculator.ShooterSolution solution;

        if (rawDistance.isPresent()) {
            // STEP A: Remove the crazy spikes
            double medianDist = outlierFilter.calculate(rawDistance.get());
            // STEP B: Smooth the tiny vibrations
            filteredDistance = smoothFilter.calculate(medianDist);

            solution = ShooterAimCalculator.solve(filteredDistance);
        } else {
            // If we lose the target, keep using the last good distance for a split second
            // or use fallback
            solution = ShooterAimCalculator.fallback();
        }

        // Apply outputs
        hood.applyAngle(solution.hoodAngle());
        shooter.applyRPM(solution.rpm());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}