package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase {

    // ------------------------------------------------
    // CONSTANTS
    // ------------------------------------------------
    private static final double MAX_RPM = 6000.0;

    private static final double WHEEL_RADIUS =
        Inches.of(4).in(Meters);

    private static final double SHOOTER_HEIGHT = 0.50;
    private static final double TARGET_HEIGHT = 2.64;
    private static final double GRAVITY = 9.81;

    // ------------------------------------------------
    // HARDWARE
    // ------------------------------------------------
    private final SparkMax shooterMotor =
        new SparkMax(26, MotorType.kBrushless);

    private final SmartMotorControllerConfig shooterMotorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)

            // PID (real + sim)
            .withClosedLoopController(
                0.00015, 0, 0,
                RPM.of(MAX_RPM),
                RotationsPerSecondPerSecond.of(1200)
            )
            .withSimClosedLoopController(
                0.00015, 0, 0,
                RPM.of(MAX_RPM),
                RotationsPerSecondPerSecond.of(1200)
            )

            // Feedforward (real + sim)
            .withFeedforward(
                new SimpleMotorFeedforward(0.25, 0.12, 0.015)
            )
            .withSimFeedforward(
                new SimpleMotorFeedforward(0.25, 0.12, 0.015)
            )

            // ✅ 12:1 GEARBOX (PER TUTORIAL)
            .withGearing(
                new MechanismGearing(
                    GearBox.fromReductionStages(3, 4)
                )
            )

            .withIdleMode(MotorMode.COAST)
            .withMotorInverted(false)
            .withStatorCurrentLimit(Amps.of(40))
            .withTelemetry(
                "ShooterMotor",
                TelemetryVerbosity.HIGH
            );

    private final SmartMotorController shooterSMC =
        new SparkWrapper(
            shooterMotor,
            DCMotor.getNEO(1),
            shooterMotorConfig
        );

    // ------------------------------------------------
    // MECHANISM
    // ------------------------------------------------
    private final FlyWheel flywheel =
        new FlyWheel(
            new FlyWheelConfig(shooterSMC)
                .withDiameter(Inches.of(4))
                .withMass(Pounds.of(1)) // ✅ IMPORTANT FOR SIM
                .withUpperSoftLimit(RPM.of(MAX_RPM))
                .withTelemetry(
                    "ShooterMech",
                    TelemetryVerbosity.HIGH
                )
        );

    // ------------------------------------------------
    // DEPENDENCIES
    // ------------------------------------------------
    private final HoodSubsystem hood;

    // ------------------------------------------------
    // STATE
    // ------------------------------------------------
    private double lastDistanceMeters = 0.0;
    private double lastTargetRPM = 0.0;

    public ShooterSubsystem(HoodSubsystem hood) {
        this.hood = hood;
    }

    // ------------------------------------------------
    // SHOOT FOR DISTANCE
    // ------------------------------------------------
    public void shootForDistance(double distanceMeters) {
        lastDistanceMeters = distanceMeters;

        Angle hoodAngle = hood.getAngleForDistance(distanceMeters);
        hood.setTargetAngle(hoodAngle);

        double theta = hoodAngle.in(Radians);
        double h = TARGET_HEIGHT - SHOOTER_HEIGHT;
        double d = distanceMeters;

        double denom =
            2 * Math.cos(theta) * Math.cos(theta)
            * (h - d * Math.tan(theta));

        if (denom >= 0) {
            flywheel.setSpeed(RPM.of(0));
            return;
        }

        double v =
            Math.sqrt((GRAVITY * d * d) / (-denom));

        double rpm =
            (v / WHEEL_RADIUS) * 60.0 / (2 * Math.PI);

        lastTargetRPM = Math.min(rpm, MAX_RPM);
        flywheel.setSpeed(RPM.of(lastTargetRPM));
    }

    public LinearVelocity getExitVelocity() {
    double mps =
        flywheel.getSpeed().in(RadiansPerSecond)
        * WHEEL_RADIUS;

    return MetersPerSecond.of(mps);
}

    // ------------------------------------------------
    // TRAJECTORY (ADVANTAGESCOPE)
    // ------------------------------------------------
    public Translation3d[] getTrajectory() {
        Translation3d[] points = new Translation3d[40];

        double v =
            flywheel.getSpeed().in(RadiansPerSecond)
            * WHEEL_RADIUS;

        double theta = hood.getAngle().in(Radians);

        double vx = v * Math.cos(theta);
        double vz = v * Math.sin(theta);

        for (int i = 0; i < points.length; i++) {
            double t = i * 0.05;
            points[i] =
                new Translation3d(
                    vx * t,
                    0,
                    SHOOTER_HEIGHT
                        + vz * t
                        - 0.5 * GRAVITY * t * t
                );
        }

        return points;
    }

    public double getFlywheelSpeedMps() {
    return flywheel.getSpeed().in(RadiansPerSecond)
           * WHEEL_RADIUS;
}

    // ------------------------------------------------
    // UTIL
    // ------------------------------------------------
    public void stop() {
        flywheel.setSpeed(RPM.of(0));
    }

    
        // ================= MANUAL CONTROL =================

    /** Spin shooter at a safe fixed RPM (teleop) */
    public void spinUp() {
        flywheel.setSpeed(RPM.of(4000));
    }

    /** Placeholder for feeder — safe no-op for now */
    public void feed() {
        // If you add a feeder motor later, control it here
        Logger.recordOutput("Shooter/Feeding", true);
    }



    // ------------------------------------------------
    // PERIODIC
    // ------------------------------------------------
    @Override
    public void periodic() {
        Logger.recordOutput(
            "Shooter/DistanceMeters",
            lastDistanceMeters
        );
        Logger.recordOutput(
            "Shooter/HoodAngleDeg",
            hood.getAngle().in(Degrees)
        );
        Logger.recordOutput(
            "Shooter/TargetRPM",
            lastTargetRPM
        );
        Logger.recordOutput(
            "Shooter/ActualRPM",
            flywheel.getSpeed().in(RPM)
        );

        flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }
}
