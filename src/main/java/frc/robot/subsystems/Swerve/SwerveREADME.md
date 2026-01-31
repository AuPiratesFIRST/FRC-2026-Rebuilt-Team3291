#  YAGSL Swerve Drive Manual
## Team 3291 - 2026 Season

---

## 📋 Table of Contents
- [Overview](#-overview)
- [What You DON'T Need](#-what-you-dont-need)
- [Project Structure](#-project-structure)
- [Setup Guide](#-step-by-step-setup-guide)
- [Simulation Testing](#-simulation-testing-no-robot-needed)
- [Troubleshooting](#-troubleshooting-guide)

---

## 🎯 Overview

> **Library:** YAGSL (Yet Another General Swerve Library)  
> **Goal:** Drive a Swerve Robot using **Kraken X60** (Drive) and **NEO** (Angle) motors  
> Configuration over Code - Define your robot in JSON, let the library handle the math!

### 🔧 Hardware Stack
| Component | Model | Purpose |
|-----------|-------|---------|
| **Drive Motors** | Kraken X60 | Wheel propulsion |
| **Angle Motors** | NEO (SparkMax) | Module rotation |
| **Encoders** | CANCoder/Thrifty | Absolute position tracking |
| **Controller** | Logitech F310 | Driver input |
| **IMU** | Pigeon2 | Robot orientation |

### ⚡ Key Features
- ✅ **Zero Math Code** - All kinematics handled by YAGSL
- ✅ **JSON Configuration** - Easy to tune without recompiling
- ✅ **Simulation Support** - Test before robot access
- ✅ **Field-Oriented Drive** - Intuitive directional control
- ✅ **Auto-Balancing** - Built-in drift correction

---

## ❌ What You DON'T Need

The official YAGSL documentation contains "Theory" pages explaining internal implementation. **You can safely ignore these:**

| ❌ Skip This | 📝 Why You Don't Need It |
|--------------|--------------------------|
| `SwerveModule.java` | YAGSL creates module objects automatically from JSON |
| Math/Kinematics Classes | Vector calculations handled internally |
| "Fundamentals" Pages | Implementation details - not needed for configuration |
| Custom Odometry Code | Built into the library |

> 💡 **Remember:** You're using YAGSL as a black box. Focus on **configuration**, not implementation.

---

## 📁 Project Structure

>  **Important:** Folder names must match exactly. Wrong names = crashes!

```
src/main/deploy/
└── swerve/                          📂 MUST BE NAMED "swerve"
    ├── swervedrive.json             🔧 Robot dimensions & Gyro settings
    ├── controllerproperties.json    🎮 Joystick deadbands & sensitivity
    ├── pidfproperties.json          ⚙️  Motor PID tuning values
    └── modules/                     📦 Individual module configurations
        ├── frontleft.json           🔩 Front-Left module specs
        ├── frontright.json          🔩 Front-Right module specs
        ├── backleft.json            🔩 Back-Left module specs
        └── backright.json           🔩 Back-Right module specs
```

### 📄 JSON File Descriptions

#### `swervedrive.json` - The Robot Blueprint
```json
{
  "wheelBase": 0.5461,           // Distance between front/back wheels (meters)
  "trackWidth": 0.5461,          // Distance between left/right wheels (meters)
  "maxSpeed": 4.5,               // Maximum drive speed (m/s)
  "optionsModifierPort": 0,      // Optional slow-mode button
  "imu": {
    "type": "Pigeon2",              // Gyro type
    "id": 0,                     // CAN ID
    "canbus": "rio"              // CAN bus name
  }
}
```

#### `pidfproperties.json` - Motor Response Tuning
```json
{
  "drive": {
    "p": 0.05,                   // Proportional gain for drive
    "i": 0.0,                    // Integral (usually 0)
    "d": 0.0,                    // Derivative (usually 0)
    "f": 0.0                     // Feed-forward
  },
  "angle": {
    "p": 0.01,                   // Proportional gain for steering
    "i": 0.0,
    "d": 0.0,
    "f": 0.0
  }
}
```

#### `frontleft.json` (Example Module)
```json
{
  "drive": {
    "type": "krakenx60",
    "id": 1,
    "canbus": "canivore"
  },
  "angle": {
    "type": "sparkmax",
    "id": 5,
    "canbus": "rio"
  },
  "encoder": {
    "type": "cancoder",
    "id": 9,
    "canbus": "canivore"
  },
  "absoluteEncoderOffset": 0,    //  Set to 0 initially, calibrate later!
  "location": {
    "front": 0.273,              // Distance from center (meters)
    "left": 0.273
  }
}
```

---

## 🚀 Step-by-Step Setup Guide

### 📖 Phase 1: Configuration (Computer Work)

> 🎯 **Objective:** Generate and customize JSON configuration files

#### Step 1.1: Generate Base Configs
1. Visit the **YAGSL Config Generator**: [https://broncbotz3291.github.io/YAGSL-Example/](https://broncbotz3291.github.io/YAGSL-Example/)
2. Select hardware:
   - Drive: **Kraken X60**
   - Angle: **NEO (SparkMax)**
   - Encoder: **CANCoder** (or your actual encoder type)
3. Enter robot dimensions (measure with tape measure!)
4. Download the generated files

#### Step 1.2: Apply the "Zero Rule"
>  **First Step**

In **ALL 4 module JSON files**, set:
```json
"absoluteEncoderOffset": 0
```

**Why?** You cannot measure the physical offset until:
- ✅ Robot is fully assembled
- ✅ Electronics are powered on
- ✅ Wheels can rotate freely

We'll calibrate these values in Phase 3.

#### Step 1.3: Verify Folder Structure
```bash
# Run this command in your terminal to check structure:
ls src/main/deploy/swerve/
```

Expected output:
```
swervedrive.json
controllerproperties.json
pidfproperties.json
modules/
```

---

### 💻 Phase 2: The Code (Software Wrapper)

> 🎯 **Objective:** Create Java classes that load JSON and control the robot

We use **2 key files** to bridge YAGSL and our robot:

#### File A: `SwerveSubsystem.java` 

**Purpose:** Initialize YAGSL, read JSON configs, expose drive methods

**Key Implementation Details:**

```java
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    
    //  Manual speed limit to prevent config errors
    private final double maximumSpeed = 4.5; // meters/second
    
    public SwerveSubsystem(File directory) {
        // 📂 Load all JSON files from deploy/swerve/
        swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
        
        // 🎮 Simulation mode check
        if (RobotBase.isSimulation()) {
            swerveDrive.setHeadingCorrection(false); // Disable drift correction
        }
    }
    
    //  Main drive method (called from RobotContainer)
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }
}
```

**Special Features:**
- ✨ **Simulation Guard:** Automatically disables features that cause simulation issues
- 🛡️ **Safety Limit:** Hardcoded max speed prevents JSON typos from causing runaway
- 📊 **Telemetry:** Built-in pose publishing to SmartDashboard

#### File B: `RobotContainer.java`

**Purpose:** Connect Xbox controller to drive commands

**Key Implementation:**

```java
public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final SwerveSubsystem swerveSubsystem;
    
    public RobotContainer() {
        // 📂 Locate the config folder
        File swerveDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        swerveSubsystem = new SwerveSubsystem(swerveDirectory);
        
        configureDefaultCommands();
    }
    
    private void configureDefaultCommands() {
        swerveSubsystem.setDefaultCommand(
            new RunCommand(() -> {
                //  Invert Y-axis so UP = FORWARD
                double xSpeed = -driverController.getLeftY();
                double ySpeed = -driverController.getLeftX();
                double rot = -driverController.getRightX();
                
                //  Apply deadband to prevent drift
                xSpeed = Math.abs(xSpeed) < 0.1 ? 0 : xSpeed;
                ySpeed = Math.abs(ySpeed) < 0.1 ? 0 : ySpeed;
                rot = Math.abs(rot) < 0.1 ? 0 : rot;
                
                Translation2d translation = new Translation2d(xSpeed, ySpeed)
                    .times(swerveSubsystem.getMaximumVelocity());
                    
                swerveSubsystem.drive(translation, rot, true); // Field-relative
            }, swerveSubsystem)
        );
    }
}
```

**Why These Inversions?**
- Xbox controllers report **DOWN as positive Y**
- We want **UP = robot moves forward**
- Solution: Multiply by `-1`

---

### 🔬 Phase 3: Tuning (On the Real Robot)

> 🎯 **Objective:** Calibrate encoder offsets and tune PID values for optimal performance

#### Step 3A: Zeroing the Encoder Offsets 

**Why This Matters:**  
Absolute encoders report positions in degrees (0-360°). Your modules need to know "what encoder value = wheels pointing forward?"

**Procedure:**

1. **Lift Robot** 
   ```
   Place robot on blocks so all wheels are off the ground
   Safety: Disable motors before approaching robot
   ```

2. **Deploy Initial Code** 
   ```bash
   ./gradlew deploy
   ```

3. **Align Modules Physically** 📐
   - Manually rotate each module so wheels point **perfectly forward**
   - Use a **straight edge** (ruler/level) to verify alignment
   - Pro tip: Bevel gears should all face the same direction (usually left or right)

4. **Read Raw Encoder Values** 
   - Open **SmartDashboard** or **AdvantageScope**
   - Navigate to: `SmartDashboard → SwerveSubsystem → Raw Encoders`
   - Note down values for all 4 modules:
     ```
     Front-Left:  143.7°
     Front-Right: 271.3°
     Back-Left:   89.2°
     Back-Right:  12.5°
     ```

5. **Update JSON Files** 
   - Open each module JSON (`frontleft.json`, etc.)
   - Replace `"absoluteEncoderOffset": 0` with your measured values:
     ```json
     "absoluteEncoderOffset": 143.7
     ```

6. **Redeploy & Test** 🧪
   ```bash
   ./gradlew deploy
   ```
   - Enable **Teleop** mode
   - Wheels should now remain forward when not commanded
   - If a module spins 180°, add/subtract 180 to that offset

---

#### Step 3B: PID Tuning (Eliminate Jitter & Sluggishness) ⚡

**What is PID?**  
Proportional-Integral-Derivative controller determines how aggressively motors respond to target changes.

**Symptoms Requiring Tuning:**

| 🚨 Problem | 🔍 Cause | ✅ Solution |
|-----------|---------|-----------|
| Wheels shake/oscillate | P too high | **Decrease** `angle.p` |
| Wheels turn slowly | P too low | **Increase** `angle.p` |
| Drive stutters | Drive P too high | **Decrease** `drive.p` |
| Sluggish acceleration | Drive P too low | **Increase** `drive.p` |

**Recommended Starting Values:**

Edit `deploy/swerve/pidfproperties.json`:

```json
{
  "angle": {
    "p": 0.01,    //  Start here for NEO steering
    "i": 0.0,     // Leave at 0 (not needed for swerve)
    "d": 0.0,     // Leave at 0
    "f": 0.0      // Leave at 0
  },
  "drive": {
    "p": 0.05,    //  Start here for Kraken drive
    "i": 0.0,
    "d": 0.0,
    "f": 0.0
  }
}
```

**Tuning Process:**

1. Change **ONE value at a time**
2. Redeploy code
3. Test drive for 30 seconds
4. Observe behavior
5. Adjust by **small increments** (±0.005)

**Advanced Tips:**
- Kraken motors are powerful → Need **lower P values**
- If using SparkMax in **brushless mode**, may need D term for smoothing
- Use **AdvantageKit** logging to visualize PID response curves

---

#### Step 3C: Check Motor Inversions 

**Why?** Sometimes motors are physically mounted backwards, causing inverted directions.

**Test Procedure:**

| 🎮 Controller Input | 🤖 Expected Behavior | ❌ If Wrong | 🛠️ Fix |
|---------------------|---------------------|------------|--------|
| Push stick **UP** | Robot moves **FORWARD** | Goes backward | Flip `driveInverted` in `swervedrive.json` |
| Push stick **RIGHT** | Robot moves **RIGHT** | Moves left | Invert joystick in code (already done) |
| Rotate stick **LEFT** | Robot spins **COUNTER-CLOCKWISE** | Spins clockwise | Flip `angleMotorInverted` in module JSONs |

**Example Fix for Drive Inversion:**

`swervedrive.json`:
```json
{
  "motors": {
    "drive": {
      "inverted": true  // ← Change to true if driving backwards
    }
  }
}
```

**Example Fix for Angle Inversion:**

`frontleft.json` (apply to all 4 modules):
```json
{
  "angle": {
    "type": "sparkmax",
    "id": 5,
    "inverted": true  // ← Change to true if spinning wrong way
  }
}
```

---

## 🖥️ Simulation Testing (No Robot Needed)

>  Test your code logic and practice driving **before robot access**!

### Setup Instructions

1. **Launch Simulation** 
   ```bash
   # In VS Code:
   Press F5 → Select "Simulate Robot Code"
   ```

2. **Open Simulation GUI** 🎮
   - Window should auto-launch
   

3. **Enable Teleop Mode** 🕹️
   ```
   GUI → System Joysticks → Select your controller
   GUI → Robot State → Enable "Teleop"
   ```

4. **Visualize on Field** 🗺️
   ```
   NetworkTables → SmartDashboard → Field
   Toggle "Field" widget to see robot position
   ```

### Simulation Limitations

| ✅ Works in Sim | ❌ Doesn't Work in Sim |
|----------------|----------------------|
| Code logic | Exact physics (friction, momentum) |
| Controller input | Motor response curves |
| Basic pathfinding | Battery voltage sag |
| Auto routines | Actual CAN bus timing |
| Command scheduling | Real encoder noise |

>  **Goal:** Verify code **compiles and runs**, not perfect realism.

### Common Sim Issues

**Problem:** Robot spins uncontrollably in sim  
**Solution:** Check for this in `SwerveSubsystem.java`:
```java
if (RobotBase.isSimulation()) {
    swerveDrive.setHeadingCorrection(false);
}
```

**Problem:** Controller not detected  
**Solution:** 
- Plug in controller **before** launching sim
- Check Device Manager (Windows) for driver issues
- Try different USB port

---

##  Troubleshooting Guide

### Common Crashes & Fixes

| 🚨 Error Message |  Root Cause | ✅ Solution |
|-----------------|--------------|-----------|
| `FileNotFoundException: swerve/` | Folder named incorrectly | Rename to exactly `swerve` (lowercase) in `deploy/` |
| `RuntimeException: JSON parse` | Typo in JSON file | Check for missing commas, brackets, or quotes |
| `NullPointerException: SwerveDrive` | Config files not deploying | Run `./gradlew deploy --info` to see deployment logs |
| `CAN timeout` on enable | Wrong CAN IDs or bus | Verify IDs in JSON match **Phoenix Tuner** |

---

### Behavior Issues

####  Robot Spins Wildly on Enable

**Possible Causes:**

1. **Wrong Encoder Offsets**  **Most Likely**
   - Symptoms: Modules snap to random angles immediately
   - Fix: Redo Phase 3, Step A (zeroing procedure)

2. **Inverted Angle Motor**
   - Symptoms: Modules spin 360° continuously
   - Fix: Toggle `angleMotorInverted` in module JSONs

3. **Bad Encoder Wiring**
   - Symptoms: Encoder values jump erratically in dashboard
   - Fix: Check physical connections, ensure CANcoder is powered

**Debug Steps:**
```
1. Open SmartDashboard
2. Monitor: Swerve → Module States → Desired vs Actual Angles
3. If Desired = 0° but Actual = 180°, you have a 180° offset error
4. Add 180 to that module's absoluteEncoderOffset
```

---

#### ↔️ Directions Are Swapped

| 🎮 Input | 🤖 Wrong Output | 🛠️ Fix |
|---------|----------------|--------|
| Forward | Backward | Invert in **code**: `-driverController.getLeftY()` |
| Right | Left | Invert in **code**: `-driverController.getLeftX()` |
| Rotate left | Rotate right | Flip `angleMotorInverted` in **JSON** |

**Where to Change Joystick Inversions:**  
[RobotContainer.java](RobotContainer.java#L45) (approximate location)

---

####  Robot Drives Slowly

**Checklist:**

- [ ] `maximumSpeed` in `SwerveSubsystem.java` is too low (should be ~4.5)
- [ ] Joystick deadband too aggressive (check `controllerproperties.json`)
- [ ] Battery voltage low (charge battery!)
- [ ] Drive motors inverted (wheels fighting each other)
- [ ] PID `f` (feedforward) term needed for Kraken:
  ```json
  "drive": {
    "p": 0.05,
    "f": 0.047  // ← Add this for better velocity tracking
  }
  ```

---

####  Wheels Don't Point Same Direction

**Symptom:** When driving straight, modules point at different angles  
**Cause:** Kinematics expects all modules to be the same distance from center

**Fix:** Measure and update in `swervedrive.json`:
```json
{
  "wheelBase": 0.5461,    // ← Must match actual robot!
  "trackWidth": 0.5461    // ← Measure twice, code once
}
```

Use a tape measure to verify:
- `wheelBase`: Distance between front/back wheel contact patches
- `trackWidth`: Distance between left/right wheel contact patches

---

###  Still Stuck?

**Resources:**

1. **YAGSL Documentation:** [https://yagsl.gitbook.io/](https://yagsl.gitbook.io/)
2. **Chief Delphi Forums:** [https://www.chiefdelphi.com/](https://www.chiefdelphi.com/)
3. **Team 3291 Discord:** Ask in `#programming` channel
4. **WPILib Examples:** [https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples](https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples)

**Debugging Checklist:**

- [ ] Code compiles without errors
- [ ] All JSON files deploy successfully (`build/deploy/swerve/`)
- [ ] CAN devices show in **Phoenix Tuner** / **REV Hardware Client**
- [ ] Absolute encoder offsets calibrated with wheels forward
- [ ] Motor inversions tested in Teleop
- [ ] Battery fully charged (>12.5V)
- [ ] Driver Station shows green comms light

---

## 📚 Quick Reference

### File Change Frequency

| 📄 File | 🔄 How Often Changed |
|---------|---------------------|
| `swervedrive.json` | Once (during initial setup) |
| Module JSONs | Once per build (encoder offsets) |
| `pidfproperties.json` | Occasionally (when tuning) |
| `controllerproperties.json` | Rarely (driver preference) |
| `SwerveSubsystem.java` | Rarely (new features only) |
| `RobotContainer.java` | Often (button mappings) |

### CAN ID Mapping Example

| Component | CAN ID | Bus |
|-----------|--------|-----|
| Front-Left Drive | 1 | CANivore |
| Front-Right Drive | 2 | CANivore |
| Back-Left Drive | 3 | CANivore |
| Back-Right Drive | 4 | CANivore |
| Front-Left Angle | 5 | RIO |
| Front-Right Angle | 6 | RIO |
| Back-Left Angle | 7 | RIO |
| Back-Right Angle | 8 | RIO |
| Front-Left Encoder | 9 | CANivore |
| Front-Right Encoder | 10 | CANivore |
| Back-Left Encoder | 11 | CANivore |
| Back-Right Encoder | 12 | CANivore |

---

## 🏆 Best Practices

1. ✅ **Always test in simulation first**
2. ✅ **Commit JSON configs to Git after calibration**
3. ✅ **Label physical modules (FL, FR, BL, BR) on robot**
4. ✅ **Keep a "known good" backup of working JSONs**
5. ✅ **Document any non-standard inversions**
6. ✅ **Use AdvantageKit for logging during competitions**
7. ✅ **Recalibrate encoder offsets if modules are disassembled**

---

**Last Updated:** January 2026  
**YAGSL Version:** 2026.1.20

---
