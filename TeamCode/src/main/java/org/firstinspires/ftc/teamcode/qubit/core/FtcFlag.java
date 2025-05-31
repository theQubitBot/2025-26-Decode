package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the flag.
 */
public class FtcFlag extends FtcSubSystemBase {
  private static final String TAG = "FtcFlag";
  public static final String FLAG_SERVO_NAME = "flagServo";
  public static final double FLAG_UP_POSITION = 0.5600;
  public static final double FLAG_DOWN_POSITION = 0.5000;
  public static final int FLAG_TRAVEL_TIME = 1200; // milliseconds
  private final boolean flagEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  private FtcServo flagServo = null;

  /**
   * Initialize standard Hardware interfaces.
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    FtcLogger.enter();
    this.telemetry = telemetry;
    if (flagEnabled) {
      flagServo = new FtcServo(hardwareMap.get(Servo.class, FLAG_SERVO_NAME));
      flagServo.setDirection(Servo.Direction.REVERSE);
      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not enabled");
    }

    FtcLogger.exit();
  }

  /**
   * Operates the flag using the gamePads.
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2) {
    FtcLogger.enter();
    FtcLogger.exit();
  }

  /**
   * Raise the flag.
   *
   * @param waitTillCompletion When true, waits till the flag is raised.
   */
  public void raise(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (flagEnabled) {
      flagServo.setPosition(FLAG_UP_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(FLAG_TRAVEL_TIME);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Lower the flag.
   *
   * @param waitTillCompletion When true, waits till the flag is lowered.
   */
  public void lower(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (flagEnabled) {
      flagServo.setPosition(FLAG_DOWN_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(FLAG_TRAVEL_TIME);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Displays intake telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (flagEnabled && telemetryEnabled && flagServo != null) {
      telemetry.addData(TAG, String.format(Locale.US, "%.4f", flagServo.getPosition()));
    }

    FtcLogger.exit();
  }

  /**
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (flagEnabled) {
      if (flagServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        flagServo.getController().pwmEnable();
      }

      lower(false);
    }

    FtcLogger.exit();
  }

  /**
   * Stops the flag.
   */
  public void stop() {
    FtcLogger.enter();
    if (flagEnabled) {
      if (flagServo != null &&
          flagServo.getController().getPwmStatus() != ServoController.PwmStatus.DISABLED) {
        flagServo.getController().pwmDisable();
      }
    }

    FtcLogger.exit();
  }
}
