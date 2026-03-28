package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class AgitatorSubsystem extends SubsystemBase {

    /*
     * =========================
     * Hardware
     * =========================
     */
    // Assign the correct CAN ID for your Kicker/Router motor
    private final SparkMax spark = new SparkMax(18, MotorType.kBrushless);

    private static final double IDLE_SPEED = -0.1;

    /*
     * ================= ========
     * Smart Motor Wrapper
     * =========================
     */
    private final SmartMotorController kickerSMC = new SparkWrapper(
            spark,
            DCMotor.getNeo550(1),
            new SmartMotorControllerConfig(this)
                    // Open Loop (Duty Cycle) just like the intake
                    .withControlMode(ControlMode.OPEN_LOOP)
                    .withFeedforward(new SimpleMotorFeedforward(0.0, 0.12, 0.0))
                    .withGearing(
                            new MechanismGearing(
                                    GearBox.fromReductionStages(5, 1)))
                    .withIdleMode(MotorMode.BRAKE) // BRAKE is better here so balls don't roll past it!
                    .withMotorInverted(true)
                    .withStatorCurrentLimit(Amps.of(30))
                    .withTelemetry("agitatorMotor", TelemetryVerbosity.LOW));

    /*
     * =========================
     * FlyWheel Mechanism (YAMS wrapper)
     * =========================
     */
    private final FlyWheelConfig agitatorConfig = new FlyWheelConfig(kickerSMC)
            .withDiameter(Inches.of(2))
            .withMass(Pounds.of(0.35))
            .withUpperSoftLimit(RPM.of(6000))
            .withTelemetry("agitatorRoller", TelemetryVerbosity.LOW);

    private final FlyWheel agitator = new FlyWheel(agitatorConfig);

    private double lastDuty = 0.0;

    public AgitatorSubsystem() {
        setDefaultCommand(stop());
    }

    /*
     * =========================
     * State API & Direct Control
     * =========================
     */
    public void setPowerDirect(double duty) {
        kickerSMC.setDutyCycle(duty);
        lastDuty = duty;
    }

    public Command set(double duty) {
        return agitator.set(duty).beforeStarting(() -> lastDuty = duty);
    }

    /*
     * =========================
     * Directional Commands
     * =========================
     */

    /** Pulls the ball DOWN into the Hopper (Matches Intake direction) */
    public Command routeToHopper() {
        // Adjust the sign depending on which way the motor is mounted!
        return set(-0.9);
    }

    public Command routeToShooter() {
        // Adjust the sign depending on which way the motor is mounted!
        return set(-0.8);
    }

    public Command idle() {
        return set(IDLE_SPEED).withName("kickerIdle");
    }

    /** Stops the kicker to hold the ball in place */
    public Command stop() {
        return set(0);
    }

    /**
     * Smart Shoot Command
     * Pushes the ball UP into the Shooter ONLY when the shooter is ready.
     * Uses the exact same flawless logic you wrote for the KitBot.
     */
    public Command smartKickerShoot(java.util.function.BooleanSupplier readyCondition) {
        return this.run(() -> {
            if (readyCondition.getAsBoolean()) {
                this.setPowerDirect(1.0); // Full send into the shooter
            } else {
                this.setPowerDirect(-0.1); // Small anti-jam hold
            }
        })
                .withName("SmartShoot")
                .finallyDo((interrupted) -> this.setPowerDirect(0.0));
    }

    /*
     * =========================
     * Required YAMS Hooks
     * =========================
     */
    @Override
    public void periodic() {
        agitator.updateTelemetry();
    }

    public double getTemperature() {
        // This returns the temperature in Celsius
        return spark.getMotorTemperature();
    }

    @Override
    public void simulationPeriodic() {
        agitator.simIterate();
    }
}