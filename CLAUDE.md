# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FTC (FIRST Tech Challenge) robot controller application for the 2025-26 DECODE competition season. The codebase is built on the official FTC SDK and contains team-specific robot control code in the `TeamCode` module, organized under the `qubit` package.

## Build System

This project uses Gradle with Android Studio. The main build configuration is split across:
- `build.gradle` - Top-level build file (rarely needs editing)
- `build.common.gradle` - Common Android build configuration shared across modules
- `build.dependencies.gradle` - Dependency management
- `TeamCode/build.gradle` - Team-specific build configuration

### Key Build Commands

Android Studio is the primary development environment. Command-line builds require JAVA_HOME to be set properly.

**Note:** This project is designed for Android Studio Ladybug (2024.2) or later, with compileSdkVersion 34, minSdkVersion 24, and targetSdkVersion 28.

### Build Variants

The project supports two build variants:
- `debug` - For development and testing
- `release` - For competition use

Before creating a release build:
1. Set build variant to `release` in Android Studio
2. Set `FtcUtils.DEBUG = false` in `FtcUtils.java`
3. Set `FtcLogger.performanceMetricsEnabled = false` in `FtcLogger.java`

## Code Architecture

### Module Structure

- **TeamCode** - Team-specific robot code (primary work area)
- **FtcRobotController** - Base FTC SDK robot controller (do not modify)
- **MeepMeep** - Path visualization tool for autonomous planning

### TeamCode Package Organization

The team code is organized under `org.firstinspires.ftc.teamcode` with the following structure:

#### `qubit` Package - Main Robot Code

- **core** - Core robot subsystems and utilities
  - **TrollBots/** - Robot configuration architecture
    - `BaseBot` - Abstract base robot class using composite pattern to manage all subsystems
    - `BotA` - Full competition robot with all subsystems enabled
    - `BotB`, `BotC`, `BotD`, `BotL` - Various test/practice robot configurations
  - `FtcDriveTrain` - Mecanum drive train control
  - `FtcCannon` - Cannon/launcher mechanism
  - `CannonControlData` - Distance-to-velocity mapping for cannon shooting
  - `FtcIntake` - Intake mechanism
  - `FtcSorter` - Game element sorting mechanism
  - `TeleOpLocalizer` - Pedro Pathing-based localization for TeleOp (goal tracking, auto-parking)
  - `FtcAprilTag` - AprilTag vision processing
  - `LlArtifactSensor` - Limelight artifact detection
  - `VpArtifactSensor` - Vision processor artifact detection
  - `FtcBlinkinLed` - LED control
  - `MatchConfig` - Match configuration persistence (alliance color, robot position, delays, etc.)
  - `FtcLogger` - Logging and performance metrics utility
  - `FtcUtils` - General utilities and constants
  - `enumerations` - Enums for alliance colors, robot positions, drive types, etc.

- **autoOps** - Autonomous operation modes
  - `AutoOp` - Main autonomous OpMode with preselectTeleOp
  - `OptionBase` - Base class for autonomous strategies
  - `OptionBlueGoal`, `OptionBlueAudience` - Blue alliance autonomous paths
  - `OptionRedGoal`, `OptionRedAudience` - Red alliance autonomous paths

- **teleOps** - TeleOp operation modes
  - `DriverTeleOp` - Main driver-controlled OpMode
  - `MatchConfigTeleOp` - Configuration interface for match settings

- **testOps** - Testing and calibration OpModes
  - Various subsystem-specific test OpModes for calibration and debugging

- **motorOps** - Motor tuning utilities
  - `MotorVelocityPIDTuner` - PID tuning for motor velocity control

#### `pedroPathing` Package - Autonomous Path Following

Integration with Pedro Pathing library for autonomous navigation:
- `Constants.java` - Configuration for follower, mecanum drive, and Pinpoint odometry
- `Tuning.java` - Path tuning utilities

Key configuration areas:
- `FollowerConstants` - Robot mass, acceleration, PID coefficients
- `MecanumConstants` - Motor names, directions, velocities
- `PinpointConstants` - Odometry sensor configuration

#### `colorSpaces` Package

Color space conversion utilities for vision processing.

### Robot Subsystem Architecture

The robot uses a **composite design pattern** with inheritance-based robot configurations:

1. `BaseBot` - Abstract base class that contains all subsystems
   - Each subsystem extends `FtcSubSystemBase` and implements `FtcSubSystemInterface`
   - Robot-level operations (init, start, stop, operate) are invoked on all subsystems
   - Factory method `BaseBot.getBot()` returns the appropriate robot instance based on `trollBot` setting

2. **Robot Implementations** - Concrete classes extending `BaseBot`:
   - `BotA` - Full competition robot (all subsystems including TeleOpLocalizer)
   - `BotB`, `BotC`, `BotD`, `BotL` - Test/practice robots with varying configurations

3. **Robot Selection** - Set via `BaseBot.trollBot` (defaults to `TrollBotEnum.TrollBotA`)

This design allows:
- Clean separation of concerns per subsystem
- Easy addition of new subsystems
- Unified lifecycle management
- Multiple robot configurations without code duplication
- Different motor directions per robot configuration

### OpMode Types and Lifecycle

**Autonomous OpModes** (`@Autonomous`):
- Extend `LinearOpMode`
- Use `runOpMode()` with `waitForStart()`
- Typically use Pedro Pathing for navigation
- Initialize with `robot = BaseBot.getBot(); robot.init(hardwareMap, telemetry, true)` (autoOp=true)

**TeleOp OpModes** (`@TeleOp`):
- Extend `OpMode`
- Implement `init()`, `init_loop()`, `start()`, `loop()`, `stop()`
- Initialize with `robot = BaseBot.getBot(); robot.init(hardwareMap, telemetry, false)` (autoOp=false)
- Use `robot.operate(gamepad1, gamepad2, lastLoopTime, runtime)` in loop

### Configuration System

The `MatchConfig` class provides persistent configuration storage:
- Alliance color (RED/BLUE)
- Robot starting position (GOAL/AUDIENCE)
- Autonomous delay
- Obelisk tag selection
- Delivery options (second row, third row)

Configuration is:
- Saved to `matchConfig.txt` on the robot controller
- Persists across OpMode switches and reboots
- Edited via `MatchConfigTeleOp`
- Read automatically during robot initialization

### TeleOp Localization

The `TeleOpLocalizer` subsystem integrates Pedro Pathing into TeleOp for:

**Goal Tracking**:
- `getGoalDistance()` - Returns distance to goal based on alliance/position
- `getGoalHeading()` - Returns heading to goal in radians
- `isValidShootingDistance()` - Checks if distance is within valid shooting range

**Driver Assistance** (via gamepad controls):
- D-pad Up: Auto-turn to face goal
- D-pad Down: Hold current position
- Share button (during endgame): Auto-park at designated parking position

**Distance Source**:
- Controlled by `CannonControlData.useCameraForGoalDistance` flag
- `false` (default): Uses Pedro Pathing localizer distance
- `true`: Uses camera-based distance measurement

The localizer initializes starting pose from `MatchConfig` and sets goal/parking poses based on alliance color and robot position.

### Vision Systems

The robot supports multiple vision systems:
1. **AprilTag** - Using FTC SDK AprilTag library for localization
2. **Limelight** - Hardware-accelerated vision processor for artifact detection
3. **Vision Processor (OpenCV)** - Custom pipelines for color-based object detection

Vision pipelines include:
- `SampleDetectionPipeline` - Detecting game samples
- `ManualObjectDetectionPipeline` - Manual object detection
- `MultipleObjectDetectionPipeline` - Multiple object tracking
- `DrawRectPipeline` - Visual feedback pipeline

### Async Updaters

To improve performance, certain subsystems use async updaters:
- `FtcAprilTagAsyncUpdater` - Async AprilTag updates
- `FtcSorterAsyncUpdater` - Async sorter state updates

These run in separate threads to avoid blocking the main control loop.

### Logging and Debugging

**FtcLogger**:
- `FtcLogger.enter()` / `FtcLogger.exit()` - Method entry/exit logging
- Performance metrics tracking (when enabled)
- Debug logging controlled by `FtcUtils.DEBUG`

**FtcUtils.DEBUG**:
- Controls debug logging throughout the codebase
- Should be `false` for competition builds
- Enables telemetry and verbose logging when `true`

### Telemetry Management

Each subsystem has a `telemetryEnabled` flag. During competition:
- Telemetry is disabled in autonomous to maximize performance
- Telemetry can be enabled for debugging via `robot.enableTelemetry()`

## Development Workflow

### Adding New OpModes

1. Create new class in appropriate package (autoOps/teleOps/testOps)
2. Extend `LinearOpMode` (auto) or `OpMode` (teleop)
3. Add `@Autonomous` or `@TeleOp` annotation with name and group
4. Initialize robot: `robot = BaseBot.getBot(); robot.init(hardwareMap, telemetry, autoOp)`
5. Remove `@Disabled` annotation to make visible in Driver Station

### Modifying Subsystems

1. Subsystem classes are in `qubit/core/`
2. Implement required interface methods from `FtcSubSystemInterface`
3. Update robot implementation (e.g., `BotA.init()`) to initialize new subsystems
4. Update `enableTelemetry()` / `disableTelemetry()` if subsystem has telemetry
5. Add subsystem to `BaseBot` as a public field
6. Handle subsystem in appropriate robot configurations (BotA, BotB, etc.)

### Tuning Autonomous Paths

1. Modify paths in `OptionBlue*/OptionRed*` classes
2. Update Pedro Pathing constants in `pedroPathing/Constants.java`
3. Use MeepMeep for path visualization before testing on robot
4. Test with `pedroPathing/Tuning.java` for path following adjustments

### Working with Vision

1. Vision pipelines are in `qubit/core/*Pipeline.java`
2. Configure color thresholds and detection parameters in pipeline classes
3. Test pipelines with test OpModes (e.g., `DrawRectPipelineTeleOp`)
4. Vision processing results are exposed through subsystem interfaces

### Cannon Control and Distance-Based Shooting

The `FtcCannon` subsystem uses distance-to-velocity mapping for accurate shooting:

1. **CannonControlData** - Stores distance ranges and corresponding velocities
   - Camera distances (C0-C125) for camera-based measurements
   - Localizer distances (L0-L125) for Pedro Pathing-based measurements
   - `useCameraForGoalDistance` flag switches between distance sources

2. **Shooting Workflow**:
   - Get distance to goal (via camera or localizer)
   - Validate distance is within acceptable range
   - Look up appropriate cannon velocity for that distance
   - Execute shot with calculated velocity

3. **Valid Shooting Ranges**:
   - Close range: 0-75 inches
   - Far range: 100-125 inches
   - Gap at 75-100 inches represents unreliable shooting zone

## Important Notes

- The FTC SDK and FtcRobotController module should not be modified
- All team code belongs in the TeamCode module under the `qubit` package
- Robot configuration is selected via `BaseBot.trollBot` - change this to switch between robot configurations
- Match configuration persists across OpMode switches - use MatchConfigTeleOp to change settings
- Bulk reads are used for sensor access optimization via `FtcBulkRead`
- Pedro Pathing requires careful initialization:
  - In autonomous: Initialize after driveTrain setup (which sets motors to run without encoders)
  - In TeleOp: `TeleOpLocalizer` must be initialized after driveTrain for proper follower setup
- The codebase uses inches for distance measurements and radians for angles in path following
- Motor directions are configured per robot in `BaseBot.getBot()` factory method
