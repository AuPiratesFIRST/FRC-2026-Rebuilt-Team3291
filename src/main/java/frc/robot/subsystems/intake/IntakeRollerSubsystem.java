package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
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

public class IntakeRollerSubsystem extends SubsystemBase {

    /*
     * =========================
     * Hardware
     * =========================
     */

    // SparkMax + NEO (built-in encoder automatically used by YAMS)
    private final SparkMax spark = new SparkMax(14, MotorType.kBrushless);

    /*
     * =========================
     * Smart Motor Config (YAMS)
     * =========================
     */

    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.OPEN_LOOP) // intake = duty control
            .withFeedforward(new SimpleMotorFeedforward(0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0))
            .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(80));

    /*
     * =========================
     * Smart Motor Wrapper
     * =========================
     */

    private final SmartMotorController intakeSMC = new SparkWrapper(
            spark,
            DCMotor.getNEO(1),
            new SmartMotorControllerConfig(this)

                    /*
                     * Control Mode
                     * Intake rollers are duty-cycle controlled.
                     */
                    .withControlMode(SmartMotorControllerConfig.ControlMode.OPEN_LOOP)

                    /*
                     * Feedforward (placeholder for sim stability)
                     */
                    .withFeedforward(
                            new SimpleMotorFeedforward(
                                    0.0,
                                    0.12,
                                    0.0))

                    /*
                     * Gearing
                     * Intake is effectively 1:1 unless you know otherwise.
                     */
                    .withGearing(
                            new MechanismGearing(
                                    GearBox.fromReductionStages(1)))

                    /*
                     * Motor Safety
                     */
                    .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
                    .withStatorCurrentLimit(Amps.of(60))

                    /*
                     * Telemetry
                     */
                    .withTelemetry(
                            "IntakeMotor",
                            SmartMotorControllerConfig.TelemetryVerbosity.MID));

    /*
     * =========================
     * FlyWheel Mechanism
     * =========================
     */

    private final FlyWheelConfig intakeConfig = new FlyWheelConfig(intakeSMC)
            .withDiameter(Inches.of(2))
            .withMass(Pounds.of(0.25))
            .withUpperSoftLimit(RPM.of(6000))
            .withTelemetry("IntakeRoller",
                    SmartMotorControllerConfig.TelemetryVerbosity.MID);

    private final FlyWheel intake = new FlyWheel(intakeConfig);

    /*
     * =========================
     * State Cache (API continuity)
     * =========================
     */

    private double lastDuty = 0.0;

    /*
     * =========================
     * Constructor
     * =========================
     */

    public IntakeRollerSubsystem() {
        setDefaultCommand(stop());
    }

    /*
     * =========================
     * Commands (YAMS pattern)
     * =========================
     */

    public Command set(double duty) {
        return intake.set(duty)
                .beforeStarting(() -> lastDuty = duty);
    }

    public Command in(double speed) {
        return set(Math.abs(speed));
    }

    public Command out(double speed) {
        return set(-Math.abs(speed));
    }

    public Command stop() {
        return set(0);
    }

    /*
     * =========================
     * State API
     * =========================
     */

    /** Returns true if intake is currently spinning */
    public boolean isRunning() {
        return Math.abs(lastDuty) > 0.05;
    }

    /** Returns true if intake is ejecting */
    public boolean outtaking() {
        return lastDuty < -0.05;
    }

    /** Current motor current (useful for note detection later) */
    public Current getCurrent() {
        return Amps.of(spark.getOutputCurrent());
    }

    /** Current duty cycle */
    public double getDutyCycle() {
        return lastDuty;
    }

    /*
     * =========================
     * Required YAMS Hooks
     * =========================
     */

    @Override
    public void periodic() {
        intake.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        intake.simIterate();
    }
}