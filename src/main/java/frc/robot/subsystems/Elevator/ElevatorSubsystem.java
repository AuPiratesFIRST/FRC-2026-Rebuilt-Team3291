package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ElevatorSubsystem extends SubsystemBase {

    // ---------------- HARDWARE ----------------
    private final SparkMax motor = new SparkMax(28, MotorType.kBrushless);
    // Through Bore Encoder on DIO 0 (Absolute mode)
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(8);

    private final double spoolDiameter = 1.2; // Inches
    private final double maxTravelMeters = 0.305; // 12 inches

    // ---------------- CONFIG ----------------
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // We do NOT use .withExternalEncoder here because SparkWrapper only supports
            // vendor-native encoders
            .withMechanismCircumference(Inches.of(spoolDiameter).times(Math.PI))
            .withClosedLoopController(
                    0, 0, 0, // Gains tuned for 17.8lb lift + 180:1 gear reduction
                    MetersPerSecond.of(2.0),
                    MetersPerSecondPerSecond.of(8.0))
            // kS = 0.2 (to break friction), kG = 0.5 (to hold robot weight), kV = 1.8
            .withFeedforward(new ElevatorFeedforward(0.2, 0.5, 0.2, 0.98))
            .withTelemetry("Elevator/Motor", TelemetryVerbosity.LOW)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(180, 1)))
            .withMotorInverted(false)
            .withStatorCurrentLimit(Amps.of(30))
            .withIdleMode(MotorMode.BRAKE);

    private final SmartMotorController smartMotor = new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);

    // ---------------- MECHANISM ----------------
    private final ElevatorConfig elevatorConfig = new ElevatorConfig(smartMotor)
            .withStartingHeight(Meters.of(0))
            .withHardLimits(Meters.of(0), Meters.of(maxTravelMeters))
            .withMass(Pounds.of(17.8))
            .withTelemetry("Elevator/Mech", TelemetryVerbosity.LOW);

    private final Elevator elevator = new Elevator(elevatorConfig);

    // ---------------- CONSTRUCTOR ----------------
    public ElevatorSubsystem() {
        // --- ABSOLUTE POSITION SEEDING ---
        // We use the DIO encoder to tell the NEO where it is at boot
        if (absoluteEncoder.isConnected()) {
            // 1. Get the raw value (e.g., 0.42)
            double rawPos = absoluteEncoder.get();

            // 2. Subtract your recorded bottom-position offset
            // Replace 0.42 with the actual number you see on your dashboard
            double offset = 0.46629376165734404;
            double correctedPos = rawPos - offset;

            // 3. Handle the "Wrap Around"
            // (If the number goes negative, the encoder passed the 1.0 -> 0.0 flip point)
            if (correctedPos < 0) {
                correctedPos += 1.0;
            }

            // 4. Calculate real meters
            double seededMeters = correctedPos * maxTravelMeters;

            // 5. Tell the motor where it is
            smartMotor.setEncoderPosition(Meters.of(seededMeters));
        }

        // Default: Pull the robot down to 0 and hold against the springs
        // setDefaultCommand(stow());
    }

    // ---------------- CALIBRATION ----------------

    /**
     * Drive elevator to physical bottom and call this to re-sync if rope stretches
     */
    public void resetZero() {
        smartMotor.setEncoderPosition(Meters.of(0));
    }

    public Command resetZeroCommand() {
        return this.runOnce(this::resetZero).withName("ResetElevatorZero");
    }

    // ---------------- COMMAND FACTORIES ----------------

    public Command stow() {
        return elevator.run(Meters.of(0)).withName("ElevatorStow");
    }

    public Command setHeight(Distance height) {
        return elevator.run(height);
    }

    public Command goToHeight(Distance height) {
        return elevator.runTo(height, Meters.of(0.01));
    }

    public Command manual(double percent) {
        return elevator.set(percent);
    }

    public double getTemperature() {
        // This returns the temperature in Celsius
        return motor.getMotorTemperature();
    }
    // ---------------- PERIODIC ----------------

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Raw Encoder Value", absoluteEncoder.get());
        elevator.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        elevator.simIterate();
    }
}