package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.*;

import org.littletonrobotics.junction.Logger;

// Commented out unused imports (motor hardware not used)
// import com.revrobotics.spark.*;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.DriverStation;
// import yams.gearing.*;
// import yams.mechanisms.config.ArmConfig;
// import yams.mechanisms.positional.Arm;
// import yams.motorcontrollers.*;
// import yams.motorcontrollers.local.SparkWrapper;

/**
 * Hood Subsystem - Represents the FIXED hood on the shooter.
 * 
 * NOTE: This robot does NOT have a motorized hood. The hood is physically
 * fixed at 65°. This subsystem exists to maintain code compatibility and
 * allow setAngle() calls to work without errors, but they are effectively no-ops.
 * 
 * The fixed angle (65°) is defined in ShooterAimCalculator.FIXED_HOOD_ANGLE.
 * If you later add a motorized hood, you can re-enable the motor control code.
 * 
 * Current Configuration:
 * - Hood is mechanically fixed at 65°
 * - setAngle() commands are accepted but do nothing (motor code disabled)
 * - Maintains API compatibility with vision-based aiming commands
 */
public class HoodSubsystem extends SubsystemBase {

    // ========== NO-OP IMPLEMENTATION (No Motor Hardware) ==========
    // Hood is physically fixed at 65° - no motor control needed
    // All motor code commented out to avoid CAN timeout errors
    
    /* ========== COMMENTED OUT MOTOR CODE ==========
    private final SparkMax hoodMotor = new SparkMax(29, SparkLowLevel.MotorType.kBrushless);

    private final SmartMotorController hoodSMC = new SparkWrapper(
            hoodMotor,
            DCMotor.getNeo550(1),
            new SmartMotorControllerConfig(this)
                    .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
                    .withClosedLoopController(
                            50.0, 0.0, 0.0,
                            DegreesPerSecond.of(180),
                            DegreesPerSecondPerSecond.of(360))
                    .withSimClosedLoopController(
                            50.0, 0.0, 0.0,
                            DegreesPerSecond.of(180),
                            DegreesPerSecondPerSecond.of(360))
                    .withFeedforward(new ArmFeedforward(0.0, 0.3, 0.02))
                    .withSimFeedforward(new ArmFeedforward(0.0, 0.3, 0.02))
                    .withGearing(
                            new MechanismGearing(
                                    GearBox.fromReductionStages(3, 4)))
                    .withStatorCurrentLimit(Amps.of(25))
                    .withSupplyCurrentLimit(Amps.of(20))
                    .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
                    .withTelemetry("HoodMotor",
                            SmartMotorControllerConfig.TelemetryVerbosity.LOW));

    private final Arm hood = new Arm(
            new ArmConfig(hoodSMC)
                    .withStartingPosition(Degrees.of(60))
                    .withSoftLimits(Degrees.of(50), Degrees.of(68))
                    .withHardLimit(Degrees.of(45), Degrees.of(70))
                    .withLength(Meters.of(0.30))
                    .withMass(Kilograms.of(2.0))
                    .withTelemetry("Hood",
                            SmartMotorControllerConfig.TelemetryVerbosity.LOW));
    ========== END COMMENTED OUT MOTOR CODE ========== */

    // Fixed hood angle (matches ShooterAimCalculator.FIXED_HOOD_ANGLE)
    private static final Angle FIXED_ANGLE = Degrees.of(65);
    private Angle lastTarget = FIXED_ANGLE;

    // ------------------------------------------------
    // COMMAND FACTORIES (NO-OP IMPLEMENTATIONS)
    // ------------------------------------------------

    public Command setAngle(Angle angle) {
        // No-op: Hood is fixed, just track requested angle for logging
        lastTarget = angle;
        return Commands.none();
    }

    public Command hold() {
        // No-op: Hood is already held at fixed position
        return Commands.none();
    }

    public Angle getAngle() {
        // Always return fixed angle
        return FIXED_ANGLE;
    }

    public void applyAngle(Angle angle) {
        // No-op: Just track requested angle for logging
        lastTarget = angle;
    }

    public Angle getTargetAngle() {
        // Return the last requested angle (for telemetry)
        return lastTarget;
    }

    // ------------------------------------------------
    // PERIODIC (MINIMAL LOGGING ONLY)
    // ------------------------------------------------

    @Override
    public void periodic() {
        // Log the fixed angle and last requested target
        Logger.recordOutput("Hood/TargetDeg", lastTarget.in(Degrees));
        Logger.recordOutput("Hood/ActualDeg", FIXED_ANGLE.in(Degrees));
        Logger.recordOutput("Hood/IsFixed", true);
    }

    @Override
    public void simulationPeriodic() {
        // No-op: No simulation needed for fixed hood
    }
}
