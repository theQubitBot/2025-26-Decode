package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the spinning wheel.
 */
public class FtcMotorSpinner extends FtcSubSystemBase {
  private static final String TAG = "FtcMotorSpinner";
  public static final String SPIN_MOTOR_NAME = "spinMotor";
  public static final double MAX_POWER = 1.0;
  public static final double MIN_POWER = -1.0;
  public static final double ZERO_POWER = 0.0;

  private final boolean spinnerEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  public FtcMotor motor = null;

  /* Constructor */
  public FtcMotorSpinner() {
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
      motor = new FtcMotor(hardwareMap.get(DcMotorEx.class, SPIN_MOTOR_NAME));
      motor.setDirection(DcMotorEx.Direction.FORWARD);
      motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      showTelemetry();
      telemetry.addData(TAG, "initialized");
    }

    FtcLogger.exit();
  }

  /**
   * Operates the wheel using the gamePads.
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2) {
    if (spinnerEnabled && motor != null) {
      double newMotorPower = motor.getPower();

      if (gamePad1.dpadUpWasPressed()) {
        newMotorPower += 0.10;
      } else if (gamePad1.dpadLeftWasPressed()) {
        newMotorPower += 0.01;
      } else if (gamePad1.dpadDownWasPressed()) {
        newMotorPower -= 0.10;
      } else if (gamePad1.dpadRightWasPressed()) {
        newMotorPower -= 0.01;
      }

      newMotorPower = Range.clip(newMotorPower, ZERO_POWER, MAX_POWER);
      motor.setPower(newMotorPower);
    }
  }

  /**
   * Displays wheel servo telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (spinnerEnabled && telemetryEnabled) {
      telemetry.addData(TAG, String.format(Locale.US, "%5.4f", motor.getPower()));
    }

    FtcLogger.exit();
  }

  /**
   * Stops the wheel.
   */
  public void stop() {
    FtcLogger.enter();
    if (spinnerEnabled) {
      motor.setPower(ZERO_POWER);
    }

    FtcLogger.exit();
  }
}
