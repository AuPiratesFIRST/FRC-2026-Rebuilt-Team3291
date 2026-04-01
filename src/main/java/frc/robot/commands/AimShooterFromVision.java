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

    private MedianFilter outlierFilter;
    private LinearFilter smoothFilter;

    private double filteredDistance = 0;
    private boolean firstRun = true;

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
    public void initialize() {
        // Resetting filters on start ensures simulation doesn't start with
        // values from the last time you ran the robot.
        outlierFilter = new MedianFilter(5);
        smoothFilter = LinearFilter.movingAverage(10);

        filteredDistance = 0;
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