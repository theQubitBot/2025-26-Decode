package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the spinning wheel.
 */
public class FtcServoSpinner extends FtcSubSystemBase {
  private static final String TAG = "FtcServoSpinner";
  public static final String SERVO_NAME = "spinServo";
  private static final double STOP_POSITION = (Servo.MIN_POSITION + Servo.MAX_POSITION) / 2.0;
  private static final double RAMP_UP_DOWN_TIME = 2.0; // seconds

  private final boolean spinnerEnabled = false;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  public FtcServo spinServo = null;

  /* Constructor */
  public FtcServoSpinner() {
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
    if (spinnerEnabled) {
      spinServo = new FtcServo(hardwareMap.get(Servo.class, SERVO_NAME));

      // Set servo directions
      spinServo.setDirection(Servo.Direction.FORWARD);
      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not enabled");
    }

    FtcLogger.exit();
  }

  /**
   * Operates the wheel using the gamePads.
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime) {
        /*
        total-time = 2sec
        loop-time = 20ms
        full-range = max position - min position = 1.0 - 0.0
        Max iterations required for full range = 2000/20 = 100
        delta change per iteration = full-range / max-iterations = (1.0 - 0.0) * loop-time / total-time
         */
    if (spinnerEnabled && spinServo != null) {
      double oldServoPosition = spinServo.getPosition();
      double newServoPosition = oldServoPosition;
      double range = Math.abs(oldServoPosition - 0.5);

      // Ensure a min increment of 0.01 if servo is at rest.
      double delta = Math.max(0.01, range) * loopTime * 1000 / RAMP_UP_DOWN_TIME;
      if (gamePad1.dpad_up) {
        newServoPosition += delta;
      } else if (gamePad1.dpad_down) {
        newServoPosition -= delta;
      }

      newServoPosition = Range.clip(newServoPosition, Servo.MIN_POSITION, Servo.MAX_POSITION);
      spinServo.setPosition(newServoPosition);
    }
  }

  /**
   * Displays wheel servo telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (spinnerEnabled && telemetryEnabled) {
      telemetry.addData(TAG, String.format(Locale.US, "%5.4f", spinServo.getPosition()));
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (spinnerEnabled) {
      spinServo.getController().pwmEnable();
      spinServo.setPosition(STOP_POSITION);
    }

    FtcLogger.exit();
  }

  /**
   * Stops the wheel.
   */
  public void stop() {
    FtcLogger.enter();
    if (spinnerEnabled) {
      spinServo.setPosition(STOP_POSITION);
      spinServo.getController().pwmDisable();
    }

    FtcLogger.exit();
  }
}
