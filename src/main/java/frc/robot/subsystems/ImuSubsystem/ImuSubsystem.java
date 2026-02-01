package frc.robot.subsystems.ImuSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class ImuSubsystem extends SubsystemBase {

    private final Pigeon2 pigeon;

    public ImuSubsystem() {
        // The CAN bus name can be an empty string if you are using the default CAN bus
        // ("rio").
        pigeon = new Pigeon2(17, "rio");

        // Create a new configuration object
        Pigeon2Configuration config = new Pigeon2Configuration();

        // Set mount pose to zero, this is the default but it's good practice to set it
        // explicitly
        config.MountPose.MountPosePitch = 0;
        config.MountPose.MountPoseRoll = 0;
        config.MountPose.MountPoseYaw = 0;

        // Apply the configuration
        pigeon.getConfigurator().apply(config);

        // Clear any sticky faults
        pigeon.clearStickyFaults();
    }

    /**
     * Exposes the Pigeon2 object for simulation purposes only.
     * 
     * @return The Pigeon2 hardware object.
     */
    public Pigeon2 getSimDevice() {
        return pigeon;
    }

    // --------------------------
    // ACCESSORS
    // --------------------------

    /** Returns the current yaw as a Rotation2d object. CCW is positive. */
    public Rotation2d getYaw() {
        // The Rotation2d constructor can directly take the Angle object from getValue()
        return new Rotation2d(pigeon.getYaw().getValue());
    }

    /** Returns robot heading as a Rotation2d (CCW positive, WPILib standard). */
    public Rotation2d getRotation2d() {
        return getYaw();
    }

    /** Returns the current yaw in radians. */
    public double getYawRad() {
        return getYaw().getRadians();
    }

    /** Returns the current yaw in degrees. */
    public double getYawDeg() {
        // Use getValueAsDouble() to get the value in the signal's native units
        // (degrees) as a double
        return pigeon.getYaw().getValueAsDouble();
    }

    /** Returns the current pitch in degrees. */
    public double getPitchDeg() {
        return pigeon.getPitch().getValueAsDouble();
    }

    /** Returns the current roll in degrees. */
    public double getRollDeg() {
        return pigeon.getRoll().getValueAsDouble();
    }

    /** Returns the Z-component of the gravity vector in g's. */
    public double getGravityVectorZ() {
        // waitForUpdate() can be used to ensure you have the latest signal
        // This is optional, but can reduce latency.
        pigeon.getGravityVectorZ().waitForUpdate(0.02); // wait up to 20ms
        return pigeon.getGravityVectorZ().getValueAsDouble();
    }

    // --------------------------
    // CONTROL
    // --------------------------

    /** Sets the current yaw to 0 degrees. */
    public void zeroYaw() {
        // Set Yaw to 0 degrees.
        // The second parameter is a timeout in seconds. 0.1s is a reasonable value.
        pigeon.setYaw(0, 0.1);
    }

    /** Sets the current yaw to a specific angle in degrees. */
    public void setYaw(double degrees) {
        pigeon.setYaw(degrees, 0.1);
    }

    // --------------------------
    // PERIODIC
    // --------------------------

    @Override
    public void periodic() {
        // In periodic, we can refresh multiple signals at once to ensure they are
        // synchronized.
        // This is especially useful if you are using fused values.
        if (RobotBase.isReal()) { // Only refresh signals on the real robot
            BaseStatusSignal.refreshAll(pigeon.getYaw(), pigeon.getPitch(), pigeon.getRoll());
        }

        // To log data, you must get the value from the StatusSignal object
        Logger.recordOutput("IMU/YawDeg", getYawDeg());
        Logger.recordOutput("IMU/PitchDeg", getPitchDeg());
        Logger.recordOutput("IMU/RollDeg", getRollDeg());
    }
}