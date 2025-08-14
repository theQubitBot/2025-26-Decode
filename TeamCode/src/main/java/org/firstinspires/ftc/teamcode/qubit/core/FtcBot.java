package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTrainEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTypeEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.TrollBotEnum;

/**
 * A class to manage the robot. This is a composite design pattern.
 * Robot level operations are simply invoked on all subsystems.
 */
public class FtcBot extends FtcSubSystemBase {
  private static final String TAG = "FtcBot";
  private boolean telemetryEnabled = true;
  public final TrollBotEnum trollBot = TrollBotEnum.TrollBotL;
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
    if (trollBot == TrollBotEnum.TrollBotA) {
      arm.telemetryEnabled = false;
      driveTrain.telemetryEnabled = false;
      blinkinLed.telemetryEnabled = false;
      flag.telemetryEnabled = false;
      imu.telemetryEnabled = false;
      intake.telemetryEnabled = false;
      lift.telemetryEnabled = false;
      rnp.telemetryEnabled = false;
    } else if (trollBot == TrollBotEnum.TrollBotB) {
    } else if (trollBot == TrollBotEnum.TrollBotC) {
      driveTrain.telemetryEnabled = false;
      imu.telemetryEnabled = false;
    } else if (trollBot == TrollBotEnum.TrollBotD) {
      driveTrain.telemetryEnabled = false;
      imu.telemetryEnabled = false;
    } else if (trollBot == TrollBotEnum.TrollBotL) {
      driveTrain.telemetryEnabled = false;
      imu.telemetryEnabled = false;
    } else {
      throw new UnsupportedOperationException("Invalid trollBot");
    }

    FtcLogger.exit();
  }

  public void enableTelemetry() {
    FtcLogger.enter();
    telemetryEnabled = true;
    if (trollBot == TrollBotEnum.TrollBotA) {
      arm.telemetryEnabled = true;
      driveTrain.telemetryEnabled = true;
      blinkinLed.telemetryEnabled = true;
      flag.telemetryEnabled = true;
      imu.telemetryEnabled = true;
      intake.telemetryEnabled = true;
      lift.telemetryEnabled = true;
      rnp.telemetryEnabled = true;
    } else if (trollBot == TrollBotEnum.TrollBotC) {
      driveTrain.telemetryEnabled = true;
      imu.telemetryEnabled = true;
    } else if (trollBot == TrollBotEnum.TrollBotD) {
      driveTrain.telemetryEnabled = true;
      imu.telemetryEnabled = true;
    } else if (trollBot == TrollBotEnum.TrollBotL) {
      driveTrain.telemetryEnabled = true;
      imu.telemetryEnabled = true;
    } else {
      throw new UnsupportedOperationException("Invalid trollBot");
    }

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

    if (trollBot == TrollBotEnum.TrollBotA) {
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

      imu = new FtcImu(this);
      imu.init(hardwareMap, telemetry);

      intake = new FtcIntake(this);
      intake.init(hardwareMap, telemetry);

      lift = new FtcLift(this);
      lift.init(hardwareMap, telemetry);

      rnp = new FtcRnp();
      rnp.init(hardwareMap, telemetry);
    } else if (trollBot == TrollBotEnum.TrollBotC) {
      bulkRead = new FtcBulkRead();
      bulkRead.init(hardwareMap, telemetry);

      config = new MatchConfig();
      config.init(hardwareMap, telemetry);

      driveTrain = new FtcDriveTrain(this);
      driveTrain.setDriveTypeAndMode(DriveTrainEnum.MECANUM_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
      driveTrain.init(hardwareMap, telemetry);

      imu = new FtcImu(this);
      imu.init(hardwareMap, telemetry);
    } else if (trollBot == TrollBotEnum.TrollBotD) {
      bulkRead = new FtcBulkRead();
      bulkRead.init(hardwareMap, telemetry);

      config = new MatchConfig();
      config.init(hardwareMap, telemetry);

      driveTrain = new FtcDriveTrain(this);
      driveTrain.setDriveTypeAndMode(DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
      driveTrain.init(hardwareMap, telemetry);

      imu = new FtcImu(this);
      imu.init(hardwareMap, telemetry);
    } else if (trollBot == TrollBotEnum.TrollBotL) {
      bulkRead = new FtcBulkRead();
      bulkRead.init(hardwareMap, telemetry);

      config = new MatchConfig();
      config.init(hardwareMap, telemetry);

      driveTrain = new FtcDriveTrain(this);
      driveTrain.setDriveTypeAndMode(DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
      driveTrain.init(hardwareMap, telemetry);

      imu = new FtcImu(this);
      imu.init(hardwareMap, telemetry);
    } else {
      throw new UnsupportedOperationException("Invalid trollBot");
    }

    telemetry.addData(TAG, "initialized");
    FtcLogger.exit();
  }

  /**
   * Operate the robot in tele operation.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
    FtcLogger.enter();

    bulkRead.clearBulkCache();
    if (trollBot == TrollBotEnum.TrollBotA) {
      blinkinLed.operate(gamePad1, gamePad2, runtime);
      arm.operate(gamePad1, gamePad2, runtime);
      driveTrain.operate(gamePad1, gamePad2, loopTime, runtime);
      flag.operate(gamePad1, gamePad2);

      intake.operate(gamePad1, gamePad2, runtime);
      lift.operate(gamePad1, gamePad2, runtime);
      rnp.operate(gamePad1, gamePad2, runtime);
    } else if (trollBot == TrollBotEnum.TrollBotC) {
      driveTrain.operate(gamePad1, gamePad2, loopTime, runtime);
    } else if (trollBot == TrollBotEnum.TrollBotD) {
      driveTrain.operate(gamePad1, gamePad2, loopTime, runtime);
    } else if (trollBot == TrollBotEnum.TrollBotL) {
      driveTrain.operate(gamePad1, gamePad2, loopTime, runtime);
    } else {
      throw new UnsupportedOperationException("Invalid trollBot");
    }

    if (telemetryEnabled) {
      if (trollBot == TrollBotEnum.TrollBotA) {
        arm.showTelemetry();
        blinkinLed.showTelemetry();
        flag.showTelemetry();
        imu.showTelemetry();
        intake.showTelemetry();
        showGamePadTelemetry(gamePad1);
        driveTrain.showTelemetry();
        lift.showTelemetry();
        rnp.showTelemetry();
      } else if (trollBot == TrollBotEnum.TrollBotB) {
        imu.showTelemetry();
        showGamePadTelemetry(gamePad1);
      } else if (trollBot == TrollBotEnum.TrollBotC) {
        imu.showTelemetry();
        showGamePadTelemetry(gamePad1);
        driveTrain.showTelemetry();
      } else if (trollBot == TrollBotEnum.TrollBotD) {
        imu.showTelemetry();
        showGamePadTelemetry(gamePad1);
        driveTrain.showTelemetry();
      } else if (trollBot == TrollBotEnum.TrollBotL) {
        imu.showTelemetry();
        showGamePadTelemetry(gamePad1);
        driveTrain.showTelemetry();
      } else {
        throw new UnsupportedOperationException("Invalid trollBot");
      }
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
    if (trollBot == TrollBotEnum.TrollBotA) {
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
    } else if (trollBot == TrollBotEnum.TrollBotB) {
    } else if (trollBot == TrollBotEnum.TrollBotC) {
    } else if (trollBot == TrollBotEnum.TrollBotD) {
    } else if (trollBot == TrollBotEnum.TrollBotL) {

    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  public void stop() {
    FtcLogger.enter();
    if (trollBot == TrollBotEnum.TrollBotA) {
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
    } else if (trollBot == TrollBotEnum.TrollBotB) {
    } else if (trollBot == TrollBotEnum.TrollBotC) {
      if (driveTrain != null) {
        driveTrain.stop();
      }

      if (imu != null) {
        imu.stop();
      }
    } else if (trollBot == TrollBotEnum.TrollBotD) {
      if (driveTrain != null) {
        driveTrain.stop();
      }

      if (imu != null) {
        imu.stop();
      }
    } else if (trollBot == TrollBotEnum.TrollBotL) {
      if (driveTrain != null) {
        driveTrain.stop();
      }

      if (imu != null) {
        imu.stop();
      }
    }

    FtcLogger.exit();
  }
}
