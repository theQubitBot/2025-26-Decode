package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the spinning wheel.
 */
public class FtcCannon extends FtcSubSystemBase {
  private static final String TAG = "FtcCannon";
  public static final String LEFT_CANNON_MOTOR_NAME = "leftCannonMotor";
  public static final String RIGHT_CANNON_MOTOR_NAME = "rightCannonMotor";
  public static final double LEFT_CANNON_FIRE_POWER = 0.75;
  public static final double RIGHT_CANNON_FIRE_POWER = 0.75;
  public static final String LEFT_TRIGGER_SERVO_NAME = "leftTriggerServo";
  public static final String RIGHT_TRIGGER_SERVO_NAME = "rightTriggerServo";
  public static final double LEFT_TRIGGER_UP_POSITION = 0.5600;
  public static final double LEFT_TRIGGER_DOWN_POSITION = 0.5050;
  public static final double RIGHT_TRIGGER_UP_POSITION = 0.4440;
  public static final double RIGHT_TRIGGER_DOWN_POSITION = 0.5020;

  private final boolean cannonEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  public FtcMotor leftCannonMotor = null, rightCannonMotor = null;
  public FtcServo leftTriggerServo = null, rightTriggerServo = null;

  /* Constructor */
  public FtcCannon() {
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
    if (cannonEnabled) {
      leftCannonMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, LEFT_CANNON_MOTOR_NAME));
      leftCannonMotor.setDirection(DcMotorEx.Direction.FORWARD);
      leftCannonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      leftCannonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      leftCannonMotor.setPower(FtcMotor.ZERO_POWER);

      rightCannonMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, RIGHT_CANNON_MOTOR_NAME));
      rightCannonMotor.setDirection(DcMotorEx.Direction.REVERSE);
      rightCannonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      rightCannonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      rightCannonMotor.setPower(FtcMotor.ZERO_POWER);

      leftTriggerServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_TRIGGER_SERVO_NAME));
      leftTriggerServo.setDirection(Servo.Direction.FORWARD);
      leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);

      rightTriggerServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_TRIGGER_SERVO_NAME));
      rightTriggerServo.setDirection(Servo.Direction.FORWARD);
      rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);

      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not initialized");
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
    if (cannonEnabled && leftCannonMotor != null && rightCannonMotor != null) {
      if (gamePad1.right_trigger >= 0.5) {
        leftCannonMotor.setPower(LEFT_CANNON_FIRE_POWER);
        rightCannonMotor.setPower(RIGHT_CANNON_FIRE_POWER);
      } else {
        leftCannonMotor.setPower(FtcMotor.ZERO_POWER);
        rightCannonMotor.setPower(FtcMotor.ZERO_POWER);
      }

      if (gamePad1.xWasPressed()) {
        leftTriggerServo.setPosition(LEFT_TRIGGER_UP_POSITION);
      } else if (gamePad1.xWasReleased()) {
        leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
      } else if (gamePad1.bWasPressed()) {
        rightTriggerServo.setPosition(RIGHT_TRIGGER_UP_POSITION);
      } else if (gamePad1.bWasReleased()) {
        rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
      }
    }
  }

  /**
   * Displays wheel servo telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (cannonEnabled && telemetryEnabled && leftCannonMotor != null && rightCannonMotor != null) {
      telemetry.addData(TAG, String.format(Locale.US, "Left: %2.1f, Right: %2.1f",
          leftCannonMotor.getPower(), rightCannonMotor.getPower()));
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (cannonEnabled) {
      if (leftTriggerServo != null &&
          leftTriggerServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        leftTriggerServo.getController().pwmEnable();
      }

      if (rightTriggerServo != null &&
          rightTriggerServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        rightTriggerServo.getController().pwmEnable();
      }
    }

    FtcLogger.exit();
  }

  /**
   * Stops the wheel.
   */
  public void stop() {
    FtcLogger.enter();
    if (cannonEnabled) {
      leftCannonMotor.setPower(FtcMotor.ZERO_POWER);
      rightCannonMotor.setPower(FtcMotor.ZERO_POWER);
      leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
      rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
    }

    FtcLogger.exit();
  }
}
