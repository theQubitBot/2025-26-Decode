package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the robot catapult.
 */
public class FtcCatapult extends FtcSubSystemBase {
  private static final String TAG = "FtcCatapult";
  public static final String CATAPULT_SERVO_NAME = "droneServo";
  public static final double SET_POSITION = 0.5900;
  public static final double RELEASE_POSITION = 0.7000;
  public static final long CATAPULT_RELEASE_TIME_MS = 700;
  public static final long CATAPULT_SET_TIME_MS = 200;
  private final boolean catapultEnabled = true;
  public boolean telemetryEnabled = true;
  private FtcBot parent = null;
  private Telemetry telemetry = null;
  public FtcServo catapultServo = null;

  /**
   * Initialize standard Hardware interfaces.
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry, FtcBot parent) {
    FtcLogger.enter();
    this.parent = parent;
    this.telemetry = telemetry;
    if (catapultEnabled) {
      catapultServo = new FtcServo(hardwareMap.get(Servo.class, CATAPULT_SERVO_NAME));
      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not enabled");
    }

    FtcLogger.exit();
  }

  /**
   * Operates the catapult using the gamePads.
   * gamePad options -> release
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
    FtcLogger.enter();
    if (catapultEnabled &&
        // Debug mode OR end game or both controllers are used
        (FtcUtils.DEBUG || runtime.seconds() > 90 || (gamePad1.start && gamePad2.start))) {
      if (gamePad1.startWasPressed() || gamePad2.startWasPressed()) {
        release(true);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Rotate to release the catapult.
   *
   * @param waitTillCompletion When true, waits till operation completes.
   */
  private void release(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (catapultEnabled) {
      if (parent == null) {
        // We are likely testing catapult
        catapultServo.setPosition(RELEASE_POSITION);
        if (waitTillCompletion) {
          FtcUtils.sleep(CATAPULT_RELEASE_TIME_MS);
        }
      } else {
        // Start this lengthy operation only if catapult is not already released.
        if (!FtcUtils.areEqual(catapultServo.getPosition(), RELEASE_POSITION, FtcUtils.EPSILON4)) {
          catapultServo.setPosition(RELEASE_POSITION);
          if (waitTillCompletion) {
            FtcUtils.sleep(CATAPULT_RELEASE_TIME_MS);
          }
        }
      }

      // reset catapult for next match
      set(false);
    }

    FtcLogger.exit();
  }

  /**
   * Rotate to set the catapult.
   *
   * @param waitTillCompletion When true, waits till operation completes.
   */
  public void set(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (catapultEnabled) {
      catapultServo.setPosition(SET_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(CATAPULT_SET_TIME_MS);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Displays catapult telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (catapultEnabled && telemetryEnabled) {
      telemetry.addData(TAG, String.format(Locale.US, "Position: %.4f",
          catapultServo.getPosition()));
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (catapultEnabled) {
      catapultServo.getController().pwmEnable();
      set(false);
    }

    FtcLogger.exit();
  }

  /**
   * Stops the catapult.
   */
  public void stop() {
    FtcLogger.enter();
    if (catapultEnabled) {
      catapultServo.getController().pwmDisable();
    }

    FtcLogger.exit();
  }
}
