package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.LinearVelocity;

import org.littletonrobotics.junction.Logger;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.*;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Shooter Subsystem - Controls the flywheel that launches game pieces.
 * 
 * This subsystem manages a single NEO motor spinning a flywheel wheel to shoot game pieces.
 * The flywheel speed is controlled in RPM (revolutions per minute).
 * 
 * Key Features:
 * - Closed-loop velocity control using YAMS (Yet Another Motor library)
 * - Configurable RPM setpoints for different shot distances
 * - Geared 3:4 reduction for higher torque
 * - Coast mode when disabled to reduce motor heating
 * - Full physics simulation support
 * 
 * The shooter uses a command factory pattern - call setRPM() to get a command
 * that will spin up the flywheel and maintain the speed.
 */
public class ShooterSubsystem extends SubsystemBase {

    // Maximum safe RPM for the flywheel (limited by motor and mechanical constraints)
    private static final double MAX_RPM = 6000.0;

    // ========== HARDWARE ==========
    // SparkMax motor controller controlling one NEO brushless motor
    // CAN ID 28 must match what's configured in REV Hardware Client
    private final SparkMax shooterMotor = new SparkMax(28, MotorType.kBrushless);

    // ========== YAMS SMART MOTOR CONTROLLER ==========
    // YAMS wraps the SparkMax and adds features like automatic logging,
    // simulation, and simplified configuration
    private final SmartMotorController shooterSMC = new SparkWrapper(
            shooterMotor,
            DCMotor.getNEO(1),  // Motor model for physics simulation
            new SmartMotorControllerConfig(this)
                    // Use closed-loop (PID) control for accurate velocity
                    .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
                    // PID gains: P=0.001 (gentle), I=0, D=0
                    // Max velocity = 6000 RPM, max acceleration = 1200 RPM/s
                    .withClosedLoopController(
                            0.001, 0.0, 0.0,
                            RPM.of(MAX_RPM),
                            RotationsPerSecondPerSecond.of(1200))
                    // Feedforward: kS=0.25V, kV=0.12V/(rad/s), kA=0.015V/(rad/s²)
                    .withFeedforward(
                            new SimpleMotorFeedforward(0.25, 0.12, 0.015))
                    // 3:4 gear reduction (motor spins faster than flywheel)
                    .withGearing(
                            new MechanismGearing(
                                    GearBox.fromReductionStages(3, 4)))
                    // Coast mode = motor freewheels when disabled (reduces heat)
                    .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
                    // Limit current to 40A to prevent brownouts
                    .withStatorCurrentLimit(Amps.of(40))
                    // Medium verbosity logging
                    .withTelemetry("ShooterMotor",
                            SmartMotorControllerConfig.TelemetryVerbosity.MID));

    private final FlyWheel flywheel = new FlyWheel(
            new FlyWheelConfig(shooterSMC)
                    .withDiameter(Inches.of(4))
                    .withMass(Pounds.of(1))
                    .withUpperSoftLimit(RPM.of(MAX_RPM))
                    .withTelemetry("ShooterMech",
                            SmartMotorControllerConfig.TelemetryVerbosity.LOW));

    private double lastTargetRPM = 0.0;

    // ------------------------------------------------
    // COMMAND FACTORIES
    // ------------------------------------------------

    public Command setRPM(double rpm) {
        return flywheel.setSpeed(() -> {
            lastTargetRPM = rpm;
            return RPM.of(rpm);
        });
    }

    public Command stop() {
        return setRPM(0);
    }

    // ------------------------------------------------
    // DIRECT APPLY (USED BY COMMANDS)
    // ------------------------------------------------
    public LinearVelocity getExitVelocity() {
        // Flywheel diameter = 4 inches
        double diameterMeters = Inches.of(4).in(Meters);
        double circumference = Math.PI * diameterMeters;

        // RPM → rotations per second
        double rps = flywheel.getSpeed().in(RPM) / 60.0;

        // v = rps × circumference
        return MetersPerSecond.of(rps * circumference);
    }

    public void applyRPM(double rpm) {
        lastTargetRPM = rpm;
        flywheel.setSpeed(RPM.of(rpm)).schedule();
    }

    public double getTargetRPM() {
        return lastTargetRPM;
    }

    public double getActualRPM() {
        return flywheel.getSpeed().in(RPM);
    }

    // ------------------------------------------------
    // PERIODIC
    // ------------------------------------------------

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/TargetRPM", lastTargetRPM);
        Logger.recordOutput("Shooter/ActualRPM",
                flywheel.getSpeed().in(RPM));

        flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }
}
