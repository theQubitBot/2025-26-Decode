package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the artifact sorter.
 */
public class FtcSorter extends FtcSubSystemBase {
  private static final String TAG = "FtcSorter";
  public static final String SORTER_SERVO_NAME = "sorterServo";
  public static final double SORTER_GREEN_POSITION = 0.4910;
  public static final double SORTER_PURPLE_POSITION = 0.4320;
  public static final double SORTER_STRAIGHT_POSITION =
      (SORTER_GREEN_POSITION + SORTER_PURPLE_POSITION) / 2;
  public static final int SORTER_SERVO_MOVE_TIME = 200; // milliseconds
  private final boolean sorterEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  public FtcServo sorterServo = null;
  private FtcSorterAsyncUpdater sorterAsyncUpdater = null;
  private final FtcBot parent;

  /* Constructor */
  public FtcSorter(FtcBot robot) {
    parent = robot;
  }

  /**
   * Initialize standard Hardware interfaces
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    FtcLogger.enter();
    this.telemetry = telemetry;
    if (sorterEnabled) {
      sorterServo = new FtcServo(hardwareMap.get(Servo.class, SORTER_SERVO_NAME));
      sorterServo.setDirection(Servo.Direction.FORWARD);
      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not enabled");
    }

    FtcLogger.exit();
  }

  /**
   * Operates the sorter automatically using artifact sensor
   */
  public void operate() {
    LlArtifactSensor.ArtifactColor artifactColor = parent.artifactSensor.getArtifactColor();
    if (artifactColor == LlArtifactSensor.ArtifactColor.GREEN) {
      setGreen(false);
    } else if (artifactColor == LlArtifactSensor.ArtifactColor.PURPLE) {
      setPurple(false);
    }
  }

  /**
   * Operates the sorter automatically using artifact sensor OR
   * using gamepads.
   *
   * @param gamePad1 Gamepad1 to use.
   * @param gamePad2 Gamepad2 to use.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2) {
    FtcLogger.enter();
    if (gamePad1.a || gamePad2.a) {
      setStraight(false);
    } else {
      operate();
    }

    FtcLogger.exit();
  }

  /**
   * Sets the sorter to the green barrel.
   *
   * @param waitTillCompletion When True, waits for sorter to complete the movement.
   */
  public void setGreen(boolean waitTillCompletion) {
    if (sorterEnabled && sorterServo != null) {
      sorterServo.setPosition(SORTER_GREEN_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(SORTER_SERVO_MOVE_TIME);
      }
    }
  }

  /**
   * Sets the sorter to the purple barrel.
   *
   * @param waitTillCompletion When True, waits for sorter to complete the movement.
   */
  public void setPurple(boolean waitTillCompletion) {
    if (sorterEnabled && sorterServo != null) {
      sorterServo.setPosition(SORTER_PURPLE_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(SORTER_SERVO_MOVE_TIME);
      }
    }
  }

  /**
   * Sets the sorter straight ahead.
   *
   * @param waitTillCompletion When True, waits for sorter to complete the movement.
   */
  public void setStraight(boolean waitTillCompletion) {
    if (sorterEnabled && sorterServo != null) {
      sorterServo.setPosition(SORTER_STRAIGHT_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(SORTER_SERVO_MOVE_TIME);
      }
    }
  }

  /**
   * Displays sorter telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (sorterEnabled && telemetryEnabled) {
      telemetry.addData(TAG, String.format(Locale.US, "%5.4f (%s)",
          sorterServo.getPosition(),
          sorterServo.getPosition() == SORTER_GREEN_POSITION ? "Green" :
              sorterServo.getPosition() == SORTER_PURPLE_POSITION ? "Purple" :
                  sorterServo.getPosition() == SORTER_STRAIGHT_POSITION ? "Straight" : "Unknown"));
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (sorterEnabled && sorterServo != null) {
      if (sorterServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        sorterServo.getController().pwmEnable();
      }

      setPurple(false);
    }

    FtcLogger.exit();
  }

  public void startAsyncUpdater() {
    if (sorterAsyncUpdater != null) {
      sorterAsyncUpdater.stop();
    }

    sorterAsyncUpdater = new FtcSorterAsyncUpdater(this);
    Thread sorterUpdaterThread = new Thread(sorterAsyncUpdater);
    sorterUpdaterThread.start();
  }

  /**
   * Stops the sorter.
   */
  public void stop() {
    FtcLogger.enter();
    if (sorterAsyncUpdater != null) {
      sorterAsyncUpdater.stop();
    }

    FtcLogger.exit();
  }
}
