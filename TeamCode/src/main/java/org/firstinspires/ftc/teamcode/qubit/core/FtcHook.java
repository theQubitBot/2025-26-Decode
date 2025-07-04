package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the robot hook.
 */
public class FtcHook extends FtcSubSystemBase {
  private static final String TAG = "FtcHook";
  public static final String HOOK_SERVO_NAME = "hookServo";
  public static final double HOOK_SERVO_HORIZONTAL_POSITION = 0.4490;
  public static final double HOOK_SERVO_VERTICAL_POSITION = 0.5360;
  private static final long HOOK_SERVO_MOVE_UP_TIME_MS = 300;
  private static final long HOOK_SERVO_MOVE_DOWN_TIME_MS = 200;
  public static final String HOOK_MOTOR_NAME = "hookMotor";
  public static final int HOOK_MOTOR_LOW_POSITION = 5;
  public static final int HOOK_MOTOR_HIGH_POSITION = 7700;
  public static final double HOOK_UP_POWER = 1.0;
  public static final double HOOK_DOWN_POWER = -HOOK_UP_POWER;
  public static final double HOOK_ZERO_POWER = 0.0;
  private final boolean hookEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  public FtcServo hookServo = null;
  public FtcMotor hookMotor = null;

  /**
   * Initialize standard Hardware interfaces.
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    FtcLogger.enter();
    this.telemetry = telemetry;
    if (hookEnabled) {
      hookServo = new FtcServo(hardwareMap.get(Servo.class, HOOK_SERVO_NAME));

      hookMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, HOOK_MOTOR_NAME));
      hookMotor.setDirection(DcMotorEx.Direction.FORWARD);
      hookMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      hookMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      hookMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      hookMotor.setTargetPosition(HOOK_MOTOR_LOW_POSITION);
      hookMotor.setPower(HOOK_ZERO_POWER);

      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not enabled");
    }

    FtcLogger.exit();
  }

  /**
   * Operates the hook using the gamePads.
   * DPad up -> raise
   * DPad down -> lower
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
    FtcLogger.enter();
    if (hookEnabled &&
        // Debug mode or end game or both controllers are used
        (FtcUtils.DEBUG || runtime.seconds() > 90 ||
            (gamePad1.dpad_up && gamePad2.dpad_up) ||
            (gamePad1.dpad_down && gamePad2.dpad_down) ||
            (gamePad1.share && gamePad2.share))) {

      if (gamePad1.shareWasPressed() || gamePad2.shareWasPressed()) {
        setHorizontal(false);
      }

      if (gamePad1.dpadUpWasPressed() || gamePad2.dpadUpWasPressed()) {
        setVertical(true);
        raise();
      } else if (gamePad1.dpadDownWasPressed() || gamePad2.dpadDownWasPressed()) {
        lower();
      }
    }

    FtcLogger.exit();
  }

  /**
   * Raises the hook by operating the hook motor.
   */
  public void raise() {
    FtcLogger.enter();
    if (hookEnabled) {
      int currentPosition = hookMotor.getCurrentPosition();
      double hookPower = HOOK_ZERO_POWER;
      if (currentPosition < HOOK_MOTOR_HIGH_POSITION) {
        // Must set motor position before setting motor mode.
        hookMotor.setTargetPosition(HOOK_MOTOR_HIGH_POSITION);
        hookMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        hookPower = HOOK_UP_POWER;
      }

      hookMotor.setPower(hookPower);
    }

    FtcLogger.exit();
  }

  /**
   * Lowers the hook by operating the hook motor.
   */
  private void lower() {
    FtcLogger.enter();
    if (hookEnabled && hookMotor != null) {
      int currentPosition = hookMotor.getCurrentPosition();
      double hookPower = HOOK_ZERO_POWER;
      if (currentPosition > HOOK_MOTOR_LOW_POSITION) {
        // Must set motor position before setting motor mode.
        hookMotor.setTargetPosition(HOOK_MOTOR_LOW_POSITION);
        hookMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        hookPower = HOOK_DOWN_POWER;
      }

      hookMotor.setPower(hookPower);
    }

    FtcLogger.exit();
  }

  /**
   * Rotate hook to make it horizontal.
   *
   * @param waitTillCompletion When true, waits till operation completes.
   */
  private void setHorizontal(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (hookEnabled && hookServo != null) {
      hookServo.setPosition(HOOK_SERVO_HORIZONTAL_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(HOOK_SERVO_MOVE_DOWN_TIME_MS);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Rotate hook to make it vertical.
   *
   * @param waitTillCompletion When true, waits till operation completes.
   */
  private void setVertical(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (hookEnabled && hookServo != null) {
      hookServo.setPosition(HOOK_SERVO_VERTICAL_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(HOOK_SERVO_MOVE_UP_TIME_MS);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Displays hook telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (hookEnabled && telemetryEnabled &&
        hookServo != null && hookMotor != null) {
      telemetry.addData(TAG, String.format(Locale.US, "Servo: %.4f, Motor: %d",
          hookServo.getPosition(), hookMotor.getCurrentPosition()));
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (hookEnabled) {
      hookServo.getController().pwmEnable();
      setHorizontal(false);
    }

    FtcLogger.exit();
  }

  /**
   * Stops the catapult.
   */
  public void stop() {
    FtcLogger.enter();
    if (hookEnabled) {
      hookServo.getController().pwmDisable();
      if (hookMotor != null) {
        hookMotor.setPower(HOOK_ZERO_POWER);
      }
    }

    FtcLogger.exit();
  }
}
