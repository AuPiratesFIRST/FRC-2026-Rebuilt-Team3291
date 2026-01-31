package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

public class AimShooterFromVision extends Command {

    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final VisionSubsystem vision;

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

        int[] validTags;

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (alliance == Alliance.Red) {
            validTags = VisionConstants.RED_HUB_TAGS;
        } else {
            validTags = VisionConstants.BLUE_HUB_TAGS;
        }

        Optional<Double> distance = vision.getDistanceToTagMeters(validTags);

        ShooterAimCalculator.ShooterSolution solution;

        // ---------------- FALLBACK LOGIC ----------------
        if (distance.isEmpty()) {
            solution = ShooterAimCalculator.fallback();
        } else {
            solution = ShooterAimCalculator.solve(distance.get());
        }

        if (!solution.valid()) {
            solution = ShooterAimCalculator.fallback();
        }

        // ---------------- APPLY OUTPUTS ----------------
        hood.applyAngle(solution.hoodAngle());
        shooter.applyRPM(solution.rpm());
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
