package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;

import org.littletonrobotics.junction.Logger;

import yams.gearing.*;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.*;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Hood Subsystem - Controls the adjustable angle hood on the shooter.
 * 
 * The hood changes the launch angle of game pieces. By adjusting the hood angle
 * along with flywheel speed, we can shoot accurately at different distances.
 * 
 * Key Features:
 * - Position control using NEO 550 motor with 3:4 gearing
 * - Soft limits (5° to 100°) prevent mechanical damage
 * - Hard limits (0° to 120°) as backup safety
 * - ArmFeedforward accounts for gravity (hood weight matters at different
 * angles)
 * - Brake mode holds position when disabled
 * 
 * The hood uses a command factory pattern - call setAngle() to get a command
 * that will move to a specific angle and hold it there.
 */
public class HoodSubsystem extends SubsystemBase {

    // ========== HARDWARE ==========
    // SparkMax controlling one NEO 550 motor (smaller than regular NEO)
    // CAN ID 29 must match REV Hardware Client configuration
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
                    .withStartingPosition(Degrees.of(20))
                    .withSoftLimits(Degrees.of(5), Degrees.of(100))
                    .withHardLimit(Degrees.of(0), Degrees.of(120))
                    .withLength(Meters.of(0.30))
                    .withMass(Kilograms.of(2.0))
                    .withTelemetry("Hood",
                            SmartMotorControllerConfig.TelemetryVerbosity.LOW));

    private Angle lastTarget = Degrees.of(20);
    private boolean zeroed = false;

    // ------------------------------------------------
    // COMMAND FACTORIES
    // ------------------------------------------------

    public Command setAngle(Angle angle) {
        return hood.setAngle(() -> {
            lastTarget = angle;
            return angle;
        });
    }

    public Command hold() {
        return hood.setAngle(this::getAngle);
    }

    public Angle getAngle() {
        return hood.getAngle();
    }

    public void applyAngle(Angle angle) {
        lastTarget = angle;
        hood.setAngle(() -> angle).schedule();
    }

    public Angle getTargetAngle() {
        return lastTarget;
    }

    // ------------------------------------------------
    // PERIODIC
    // ------------------------------------------------

    @Override
    public void periodic() {

        if (!zeroed && DriverStation.isEnabled()) {
            hoodSMC.setPosition(Degrees.zero());
            zeroed = true;
        }

        hood.updateTelemetry();

        Logger.recordOutput("Hood/TargetDeg", lastTarget.in(Degrees));
        Logger.recordOutput("Hood/ActualDeg",
                hood.getAngle().in(Degrees));
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
