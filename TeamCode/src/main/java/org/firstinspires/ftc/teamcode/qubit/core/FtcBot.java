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
  public final TrollBotEnum trollBot = TrollBotEnum.TrollBotA;

  // robot sub systems
  public FtcAprilTag aprilTag = null;
  public ArtifactSensor artifactSensor = null;
  public FtcBlinkinLed blinkinLed = null;
  public FtcBulkRead bulkRead = null;
  public FtcCannon cannon = null;
  public MatchConfig config = null;
  public FtcDriveTrain driveTrain = null;
  public FtcIntake intake = null;
  public FtcSorter sorter = null;
  private Telemetry telemetry = null;

  /* Constructor */
  public FtcBot() {
  }

  public void disableTelemetry() {
    FtcLogger.enter();
    telemetryEnabled = false;
    if (trollBot == TrollBotEnum.TrollBotA) {
      aprilTag.telemetryEnabled = false;
      artifactSensor.telemetryEnabled = false;
      blinkinLed.telemetryEnabled = false;
      cannon.telemetryEnabled = false;
      driveTrain.telemetryEnabled = false;
      intake.telemetryEnabled = false;
      sorter.telemetryEnabled = false;
    } else if (trollBot == TrollBotEnum.TrollBotB) {
    } else if (trollBot == TrollBotEnum.TrollBotC) {
      driveTrain.telemetryEnabled = false;
    } else if (trollBot == TrollBotEnum.TrollBotD) {
      driveTrain.telemetryEnabled = false;
    } else if (trollBot == TrollBotEnum.TrollBotL) {
      driveTrain.telemetryEnabled = false;
    } else {
      throw new UnsupportedOperationException("Invalid trollBot");
    }

    FtcLogger.exit();
  }

  public void enableTelemetry() {
    FtcLogger.enter();
    telemetryEnabled = true;
    if (trollBot == TrollBotEnum.TrollBotA) {
      aprilTag.telemetryEnabled = true;
      artifactSensor.telemetryEnabled = true;
      blinkinLed.telemetryEnabled = true;
      cannon.telemetryEnabled = true;
      driveTrain.telemetryEnabled = true;
      intake.telemetryEnabled = true;
      sorter.telemetryEnabled = true;
    } else if (trollBot == TrollBotEnum.TrollBotC) {
      driveTrain.telemetryEnabled = true;
    } else if (trollBot == TrollBotEnum.TrollBotD) {
      driveTrain.telemetryEnabled = true;
    } else if (trollBot == TrollBotEnum.TrollBotL) {
      driveTrain.telemetryEnabled = true;
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
   * @param autoOp      If true, initializes subsystems and takes other actions that make sense only
   *                    in autoOp. You may save on processing or not move servos during teleOp init.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    FtcLogger.enter();
    this.telemetry = telemetry;

    if (trollBot == TrollBotEnum.TrollBotA) {
      aprilTag = new FtcAprilTag(this);
      aprilTag.init(hardwareMap, telemetry);

      artifactSensor = new ArtifactSensor();
      artifactSensor.init(hardwareMap, telemetry);

      blinkinLed = new FtcBlinkinLed(this);
      blinkinLed.init(hardwareMap, telemetry);

      bulkRead = new FtcBulkRead();
      bulkRead.init(hardwareMap, telemetry);

      cannon = new FtcCannon(this);
      cannon.init(hardwareMap, telemetry);

      config = new MatchConfig();
      config.init(hardwareMap, telemetry);

      driveTrain = new FtcDriveTrain(this);
      driveTrain.setDriveTypeAndMode(DriveTrainEnum.MECANUM_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
      driveTrain.init(hardwareMap, telemetry);

      intake = new FtcIntake();
      intake.init(hardwareMap, telemetry);

      sorter = new FtcSorter(this);
      sorter.init(hardwareMap, telemetry);
    } else if (trollBot == TrollBotEnum.TrollBotC) {
      bulkRead = new FtcBulkRead();
      bulkRead.init(hardwareMap, telemetry);

      config = new MatchConfig();
      config.init(hardwareMap, telemetry);

      driveTrain = new FtcDriveTrain(this);
      driveTrain.setDriveTypeAndMode(DriveTrainEnum.MECANUM_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
      driveTrain.init(hardwareMap, telemetry);
    } else if (trollBot == TrollBotEnum.TrollBotD) {
      bulkRead = new FtcBulkRead();
      bulkRead.init(hardwareMap, telemetry);

      config = new MatchConfig();
      config.init(hardwareMap, telemetry);

      driveTrain = new FtcDriveTrain(this);
      driveTrain.setDriveTypeAndMode(DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
      driveTrain.init(hardwareMap, telemetry);
    } else if (trollBot == TrollBotEnum.TrollBotL) {
      bulkRead = new FtcBulkRead();
      bulkRead.init(hardwareMap, telemetry);

      config = new MatchConfig();
      config.init(hardwareMap, telemetry);

      driveTrain = new FtcDriveTrain(this);
      driveTrain.setDriveTypeAndMode(DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
      driveTrain.init(hardwareMap, telemetry);
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
      driveTrain.operate(gamePad1, gamePad2, loopTime, runtime);
      intake.operate(gamePad1, gamePad2, runtime);
      sorter.operate(gamePad1, gamePad2);
      cannon.operate(gamePad1, gamePad2);
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
        blinkinLed.showTelemetry();
        intake.showTelemetry();
        sorter.showTelemetry();
        cannon.showTelemetry();
        showGamePadTelemetry(gamePad1);
        driveTrain.showTelemetry();
      } else if (trollBot == TrollBotEnum.TrollBotB) {
        showGamePadTelemetry(gamePad1);
      } else if (trollBot == TrollBotEnum.TrollBotC) {
        showGamePadTelemetry(gamePad1);
        driveTrain.showTelemetry();
      } else if (trollBot == TrollBotEnum.TrollBotD) {
        showGamePadTelemetry(gamePad1);
        driveTrain.showTelemetry();
      } else if (trollBot == TrollBotEnum.TrollBotL) {
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
      if (aprilTag != null) {
        aprilTag.start();
      }

      if (artifactSensor != null) {
        artifactSensor.start();
      }

      if (blinkinLed != null) {
        blinkinLed.start();
      }

      if (cannon != null) {
        cannon.start();
      }

      if (intake != null) {
        intake.start();
      }

      if (sorter != null) {
        sorter.start();
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
      if (blinkinLed != null) {
        blinkinLed.stop();
      }

      if (driveTrain != null) {
        driveTrain.stop();
      }

      if (aprilTag != null) {
        aprilTag.stop();
      }

      if (artifactSensor != null) {
        artifactSensor.stop();
      }

      if (intake != null) {
        intake.stop();
      }

      if (sorter != null) {
        sorter.stop();
      }

      if (cannon != null) {
        cannon.stop();
      }
    } else if (trollBot == TrollBotEnum.TrollBotB) {
    } else if (trollBot == TrollBotEnum.TrollBotC) {
      if (driveTrain != null) {
        driveTrain.stop();
      }
    } else if (trollBot == TrollBotEnum.TrollBotD) {
      if (driveTrain != null) {
        driveTrain.stop();
      }
    } else if (trollBot == TrollBotEnum.TrollBotL) {
      if (driveTrain != null) {
        driveTrain.stop();
      }
    }

    FtcLogger.exit();
  }
}
