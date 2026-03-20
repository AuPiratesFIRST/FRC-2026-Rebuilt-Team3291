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
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeRollerSubsystem extends SubsystemBase {

    // 1. Hardware Definition
    private final SparkMax spark = new SparkMax(14, MotorType.kBrushless);
    private static final double IDLE_SPEED = 0.05;

    // 2. The Smart Motor Controller (The Wrapper)
    private final SmartMotorController intakeSMC;

    // 3. The FlyWheel Mechanism (The Physics/Logic)
    private final FlyWheel intake;

    private double lastDuty = 0.0;

    public IntakeRollerSubsystem() {
        // Define the configuration ONCE
        SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
                .withControlMode(SmartMotorControllerConfig.ControlMode.OPEN_LOOP)
                .withFeedforward(new SimpleMotorFeedforward(0.0, 0.12, 0.0))
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
                .withStatorCurrentLimit(Amps.of(60))
                .withTelemetry("Intake/Motor", SmartMotorControllerConfig.TelemetryVerbosity.MID);

        // Initialize the Wrapper using that config
        this.intakeSMC = new SparkWrapper(spark, DCMotor.getNEO(1), config);

        // Initialize the FlyWheel Mechanism
        FlyWheelConfig mechanismConfig = new FlyWheelConfig(intakeSMC)
                .withDiameter(Inches.of(2))
                .withMass(Pounds.of(0.25))
                .withUpperSoftLimit(RPM.of(6000))
                .withTelemetry("Intake/Roller", SmartMotorControllerConfig.TelemetryVerbosity.MID);

        this.intake = new FlyWheel(mechanismConfig);

        // Set default command to stop
        setDefaultCommand(stop());
    }

    /* ========================= Commands ========================= */

    public Command set(double duty) {
        return intake.set(duty).beforeStarting(() -> lastDuty = duty);
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

    public void setPowerDirect(double duty) {
        intakeSMC.setDutyCycle(duty);
        lastDuty = duty;
    }

    /* ========================= Logic/State ========================= */

    public boolean isRunning() {
        return Math.abs(lastDuty) > 0.05;
    }

    public Current getCurrent() {
        return Amps.of(spark.getOutputCurrent());
    }

    /**
     * A command that feeds ONLY when the shooter is ready.
     */
    public Command smartFeed(java.util.function.BooleanSupplier readyCondition) {
        return this.run(() -> {
            if (readyCondition.getAsBoolean()) {
                this.setPowerDirect(1.0);
            } else {
                this.setPowerDirect(-0.1); // Slight anti-jam kickback
            }
        })
                .withName("SmartFeed")
                .finallyDo((interrupted) -> this.setPowerDirect(0.0));
    }

    /* ========================= YAMS Hooks ========================= */

    @Override
    public void periodic() {
        intake.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        intake.simIterate();
    }
}