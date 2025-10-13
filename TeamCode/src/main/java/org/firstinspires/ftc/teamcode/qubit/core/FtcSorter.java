package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.Locale;

/**
 * A class to manage the spinning wheel.
 */
public class FtcSorter extends FtcSubSystemBase {
  private static final String TAG = "FtcSorter";
  public static final String SORTER_SERVO_NAME = "sorterServo";
  public static final double SORTER_GREEN_POSITION = 0.4910;
  public static final double SORTER_PURPLE_POSITION = 0.4320;
  public static final int SORTER_SERVO_MOVE_TIME = 200; // milliseconds
  private final boolean sorterEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  public FtcServo sorterServo = null;
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
   * Operates the merry-go-round using the gamePads.
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */

  // Possibly - have merry-go-round operate completely autonomously? using diff. kinds of sensors
  public void operate(Gamepad gamePad1, Gamepad gamePad2) {
    FtcLogger.enter();
    if (parent != null && parent.artifactSensor != null) {
      if (parent.artifactSensor.getSwatch() == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
        setGreen(false);
      }
      if (parent.artifactSensor.getSwatch() == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
        setPurple(false);
      }
    }

    FtcLogger.exit();
  }

  public void setGreen(boolean waitTillCompletion) {
    if (sorterEnabled && sorterServo != null) {
      sorterServo.setPosition(SORTER_GREEN_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(SORTER_SERVO_MOVE_TIME);
      }
    }
  }

  public void setPurple(boolean waitTillCompletion) {
    if (sorterEnabled && sorterServo != null) {
      sorterServo.setPosition(SORTER_PURPLE_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(SORTER_SERVO_MOVE_TIME);
      }
    }
  }

  /**
   * Displays motor power telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (sorterEnabled && telemetryEnabled) {
      telemetry.addData(TAG, String.format(Locale.US, "%5.4f", sorterServo.getPosition()));
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (sorterEnabled && sorterServo != null &&
        sorterServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      sorterServo.getController().pwmEnable();
    }

    setPurple(false);
    FtcLogger.exit();
  }

  /**
   * Stops the merry-go-round mechanism.
   */
  public void stop() {
    FtcLogger.enter();
    FtcLogger.exit();
  }
}
