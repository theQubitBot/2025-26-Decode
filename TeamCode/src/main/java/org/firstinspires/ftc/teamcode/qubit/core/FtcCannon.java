package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObeliskTagEnum;

import java.util.Locale;

/**
 * A class to manage the cannon.
 */
public class FtcCannon extends FtcSubSystemBase {
  private static final String TAG = "FtcCannon";
  public static final String LEFT_CANNON_MOTOR_NAME = "leftCannonMotor";
  public static final String RIGHT_CANNON_MOTOR_NAME = "rightCannonMotor";
  public static final double LEFT_CANNON_IDLE_POWER = 0.10;
  public static final double RIGHT_CANNON_IDLE_POWER = LEFT_CANNON_IDLE_POWER;
  public static final String LEFT_TRIGGER_SERVO_NAME = "leftTriggerServo";
  public static final String RIGHT_TRIGGER_SERVO_NAME = "rightTriggerServo";
  public static final double LEFT_TRIGGER_UP_POSITION = 0.5600;
  public static final double LEFT_TRIGGER_DOWN_POSITION = 0.5050;
  public static final double RIGHT_TRIGGER_UP_POSITION = 0.4440;
  public static final double RIGHT_TRIGGER_DOWN_POSITION = 0.5020;
  public static final int TRIGGER_MOVE_TIME = 300; // milliseconds

  private final boolean cannonEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  private final FtcBot parent;
  public FtcMotor leftCannonMotor = null, rightCannonMotor = null;
  public FtcServo leftTriggerServo = null, rightTriggerServo = null;

  private final double[][] powerMap = {
      {FtcAprilTag.MIN_RANGE, LEFT_CANNON_IDLE_POWER},
      {10.0, 0.45},
      {55.3, 0.45}, // 1200
      {62.1, 0.44}, // 1140
      {69.5, 0.44}, // 1140
      {77.2, 0.45}, // 1180
      {85.0, 0.45}, // 1180
      {92.4, 0.48}, // 1260
      {101.7, 0.50}, // 1280
      {113.7, 0.55}, // 1360
      {FtcAprilTag.MAX_RANGE, LEFT_CANNON_IDLE_POWER},
  };

  /* Constructor */
  public FtcCannon(FtcBot robot) {
    parent = robot;
  }

  public void fire(ObeliskTagEnum obeliskTag) {
    if (cannonEnabled) {
      if (obeliskTag == ObeliskTagEnum.GPP) {
        fireLeft(false);
        fireRight(true);
        fireRight(false);
      } else if (obeliskTag == ObeliskTagEnum.PGP) {
        fireRight(false);
        fireLeft(false);
        fireRight(false);
      } else {
        fireRight(false);
        fireRight(true);
        fireLeft(false);
      }

      idle();
    }
  }

  public void fireLeft(boolean waitTillDown) {
    if (cannonEnabled) {
      if (leftTriggerServo != null) {
        leftTriggerServo.setPosition(LEFT_TRIGGER_UP_POSITION);
        FtcUtils.sleep(TRIGGER_MOVE_TIME);
        leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
        if (waitTillDown) {
          FtcUtils.sleep(TRIGGER_MOVE_TIME);
        }
      }
    }
  }

  public void fireRight(boolean waitTillDown) {
    if (cannonEnabled) {
      if (rightTriggerServo != null) {
        rightTriggerServo.setPosition(RIGHT_TRIGGER_UP_POSITION);
        FtcUtils.sleep(TRIGGER_MOVE_TIME);
        rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
        if (waitTillDown) {
          FtcUtils.sleep(TRIGGER_MOVE_TIME);
        }
      }
    }
  }

  /**
   * Finds the closest matching power for the given distance.
   *
   * @param distance The distance to the goal april tag.
   * @return Power for the cannon motors.
   */
  private Double getClosestPower(double distance) {
    double minDifference = Math.abs(distance - powerMap[0][0]);
    double power = powerMap[0][1];
    for (int i = 1; i < powerMap.length; i++) {
      double currentDifference = Math.abs(distance - powerMap[i][0]);
      if (currentDifference < minDifference) {
        minDifference = currentDifference;
        power = powerMap[i][1];
      }
    }

    return power;
  }

  /**
   * Returns the current cannon power.
   */
  public double getPower() {
    double power = FtcMotor.ZERO_POWER;
    if (cannonEnabled) {
      if (leftCannonMotor != null) {
        power = leftCannonMotor.getPower();
      }
    }

    return power;
  }

  public void idle() {
    if (cannonEnabled) {
      leftCannonMotor.setPower(LEFT_CANNON_IDLE_POWER);
      rightCannonMotor.setPower(RIGHT_CANNON_IDLE_POWER);
    }
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
      leftCannonMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

      rightCannonMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, RIGHT_CANNON_MOTOR_NAME));
      rightCannonMotor.setDirection(DcMotorEx.Direction.REVERSE);
      rightCannonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      rightCannonMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

      leftTriggerServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_TRIGGER_SERVO_NAME));
      leftTriggerServo.setDirection(Servo.Direction.FORWARD);

      rightTriggerServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_TRIGGER_SERVO_NAME));
      rightTriggerServo.setDirection(Servo.Direction.FORWARD);

      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not initialized");
    }

    FtcLogger.exit();
  }

  /**
   * Operates the cannon using the gamePads.
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2) {
    if (cannonEnabled && leftCannonMotor != null && rightCannonMotor != null) {
      double power = 0.0;
      if (gamePad1.y || gamePad2.y) {
        power = 0.45;
      } else if (gamePad1.a || gamePad2.a) {
        power = 0.55;
      } else if (parent != null && parent.aprilTag != null) {
        power = getClosestPower(parent.aprilTag.getGoalRange());
      } else {
        power = FtcMotor.ZERO_POWER;
      }

      setPower(power);

      if (gamePad1.right_bumper || gamePad2.right_bumper) {
        // automatic firing based on motif
      } else if (gamePad1.xWasPressed() || gamePad2.xWasPressed()) {
        leftTriggerServo.setPosition(LEFT_TRIGGER_UP_POSITION);
      } else if (gamePad1.xWasReleased() || gamePad2.xWasReleased()) {
        leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
      } else if (gamePad1.bWasPressed() || gamePad2.bWasPressed()) {
        rightTriggerServo.setPosition(RIGHT_TRIGGER_UP_POSITION);
      } else if (gamePad1.bWasReleased() || gamePad2.bWasReleased()) {
        rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
      }
    }
  }

  public void setPower(double power) {
    if (cannonEnabled) {
      if (leftCannonMotor != null) {
        leftCannonMotor.setPower(power);
      }

      if (rightCannonMotor != null) {
        rightCannonMotor.setPower(power);
      }
    }
  }

  /**
   * Displays cannon telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (cannonEnabled && telemetryEnabled && leftCannonMotor != null && rightCannonMotor != null) {
      telemetry.addData(TAG, String.format(Locale.US, "Left: %2.1f (%4.0f), Right: %2.1f (%4.0f)",
          leftCannonMotor.getPower(), leftCannonMotor.getVelocity(),
          rightCannonMotor.getPower(), rightCannonMotor.getVelocity()));
      telemetry.addData(TAG, String.format(Locale.US, "Left: %s, Right: %s",
          leftTriggerServo.getPosition() == LEFT_TRIGGER_UP_POSITION ? "Up" : "Down",
          rightTriggerServo.getPosition() == RIGHT_TRIGGER_UP_POSITION ? "Up" : "Down"));
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (cannonEnabled) {
      if (leftCannonMotor != null) {
        leftCannonMotor.setPower(FtcMotor.ZERO_POWER);
      }

      if (rightCannonMotor != null) {
        rightCannonMotor.setPower(FtcMotor.ZERO_POWER);
      }

      if (leftTriggerServo != null) {
        if (leftTriggerServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
          leftTriggerServo.getController().pwmEnable();
        }

        leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
      }

      if (rightTriggerServo != null) {
        if (rightTriggerServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
          rightTriggerServo.getController().pwmEnable();
        }

        rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Stops the cannon.
   */
  public void stop() {
    FtcLogger.enter();
    if (cannonEnabled) {
      if (leftCannonMotor != null) {
        leftCannonMotor.setPower(FtcMotor.ZERO_POWER);
      }

      if (rightCannonMotor != null) {
        rightCannonMotor.setPower(FtcMotor.ZERO_POWER);
      }

      if (leftTriggerServo != null) {
        leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
      }

      if (rightTriggerServo != null) {
        rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
      }
    }

    FtcLogger.exit();
  }
}
