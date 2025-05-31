package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTrainEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTypeEnum;

/**
 * A class to manage the robot. This is a composite design pattern.
 * Robot level operations are simply invoked on all subsystems.
 */
public class FtcBot extends FtcSubSystemBase {
  private static final String TAG = "FtcBot";
  private boolean telemetryEnabled = true;
  public FtcArm arm = null;
  public FtcBulkRead bulkRead = null;
  public FtcBlinkinLed blinkinLed = null;
  public FtcDriveTrain driveTrain = null;
  public FtcFlag flag = null;

  // robot sub systems
  public FtcImu imu = null;
  public FtcIntake intake = null;
  public FtcLift lift = null;
  public FtcRnp rnp = null;
  public MatchConfig config = null;
  private Telemetry telemetry = null;

  /* Constructor */
  public FtcBot() {
  }

  public void disableTelemetry() {
    FtcLogger.enter();
    telemetryEnabled = false;
    arm.telemetryEnabled = false;
    driveTrain.telemetryEnabled = false;
    blinkinLed.telemetryEnabled = false;
    flag.telemetryEnabled = false;
    imu.telemetryEnabled = false;
    intake.telemetryEnabled = false;
    lift.telemetryEnabled = false;
    rnp.telemetryEnabled = false;
    FtcLogger.exit();
  }

  public void enableTelemetry() {
    FtcLogger.enter();
    telemetryEnabled = true;
    arm.telemetryEnabled = true;
    driveTrain.telemetryEnabled = true;
    blinkinLed.telemetryEnabled = true;
    flag.telemetryEnabled = true;
    imu.telemetryEnabled = true;
    intake.telemetryEnabled = true;
    lift.telemetryEnabled = true;
    rnp.telemetryEnabled = true;
    FtcLogger.exit();
  }

  /**
   * Initialize standard Hardware interfaces
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   * @param autoOp      If true, initializes the webcams. Otherwise initializes only IMU.
   *                    Typically you would save on processing by not initializing the webcams or IMU
   *                    unnecessarily
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    FtcLogger.enter();
    this.telemetry = telemetry;

    arm = new FtcArm(this);
    arm.init(hardwareMap, telemetry);

    bulkRead = new FtcBulkRead();
    bulkRead.init(hardwareMap, telemetry);

    blinkinLed = new FtcBlinkinLed(this);
    blinkinLed.init(hardwareMap, telemetry);

    config = new MatchConfig();
    config.init(hardwareMap, telemetry);

    driveTrain = new FtcDriveTrain(this);
    driveTrain.setDriveTypeAndMode(DriveTrainEnum.MECANUM_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
    driveTrain.init(hardwareMap, telemetry);

    flag = new FtcFlag();
    flag.init(hardwareMap, telemetry);

    imu = new FtcImu();
    imu.init(hardwareMap, telemetry);

    intake = new FtcIntake(this);
    intake.init(hardwareMap, telemetry);

    lift = new FtcLift(this);
    lift.init(hardwareMap, telemetry);

    rnp = new FtcRnp();
    rnp.init(hardwareMap, telemetry);

    telemetry.addData(TAG, "initialized");
    FtcLogger.exit();
  }

  /**
   * Operate the robot in tele operation.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
    FtcLogger.enter();

    bulkRead.clearBulkCache();
    blinkinLed.operate(gamePad1, gamePad2, runtime);

    // Drive operation
    arm.operate(gamePad1, gamePad2, runtime);
    driveTrain.operate(gamePad1, gamePad2, loopTime, runtime);
    flag.operate(gamePad1, gamePad2);

    intake.operate(gamePad1, gamePad2, runtime);
    lift.operate(gamePad1, gamePad2, runtime);
    rnp.operate(gamePad1, gamePad2, runtime);
    if (telemetryEnabled) {
      arm.showTelemetry();
      blinkinLed.showTelemetry();
      flag.showTelemetry();
      imu.showTelemetry();
      intake.showTelemetry();
      showGamePadTelemetry(gamePad1);
      driveTrain.showTelemetry();
      lift.showTelemetry();
      rnp.showTelemetry();
    }

    FtcLogger.exit();
  }

  /**
   * Display game pad telemetry.
   *
   * @param gamePad The gamePad.
   */
  public void showGamePadTelemetry(Gamepad gamePad) {
    FtcLogger.enter();
    if (telemetryEnabled) {
      telemetry.addData("LeftStick", "%.2f %.2f",
          gamePad.left_stick_x, gamePad.left_stick_y);
      telemetry.addData("RightStick", "%.2f, %.2f",
          gamePad.right_stick_x, gamePad.right_stick_y);
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (arm != null) {
      arm.start();
    }

    if (flag != null) {
      flag.start();
    }

    if (intake != null) {
      intake.start();
    }

    if (rnp != null) {
      rnp.start();
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  public void stop() {
    FtcLogger.enter();
    if (arm != null) {
      arm.stop();
    }

    if (blinkinLed != null) {
      blinkinLed.stop();
    }

    if (driveTrain != null) {
      driveTrain.stop();
    }

    if (flag != null) {
      flag.stop();
    }

    if (imu != null) {
      imu.stop();
    }

    if (lift != null) {
      lift.stop();
    }

    if (intake != null) {
      intake.stop();
    }

    if (rnp != null) {
      rnp.stop(false);
    }

    FtcLogger.exit();
  }
}
