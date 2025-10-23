package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObeliskTagEnum;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * A class to manage the cannon.
 */
public class FtcCannon extends FtcSubSystemBase {
  private static final String TAG = "FtcCannon";
  public static final String LEFT_CANNON_MOTOR_NAME = "leftCannonMotor";
  public static final String RIGHT_CANNON_MOTOR_NAME = "rightCannonMotor";
  public static final double LEFT_CANNON_IDLE_POWER = 0.05;
  public static final double RIGHT_CANNON_IDLE_POWER = LEFT_CANNON_IDLE_POWER;
  public static final long CANNON_RAMP_UP_TIME = 3000; // milliseconds
  public static final String LEFT_TRIGGER_SERVO_NAME = "leftTriggerServo";
  public static final String RIGHT_TRIGGER_SERVO_NAME = "rightTriggerServo";
  public static final double LEFT_TRIGGER_UP_POSITION = 0.5500;
  public static final double LEFT_TRIGGER_DOWN_POSITION = 0.5050;
  public static final double RIGHT_TRIGGER_UP_POSITION = 0.4575;
  public static final double RIGHT_TRIGGER_DOWN_POSITION = 0.5020;
  public static final int TRIGGER_MOVE_TIME = 300; // milliseconds

  private final boolean cannonEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  private final FtcBot parent;
  public FtcMotor leftCannonMotor = null, rightCannonMotor = null;
  public FtcServo leftTriggerServo = null, rightTriggerServo = null;

  public List<CannonPowerData> powerData = new ArrayList<>(10);

  /* Constructor */
  public FtcCannon(FtcBot robot) {
    parent = robot;
    powerData.add(new CannonPowerData(0, 0.05, 100)); // idle
    powerData.add(new CannonPowerData(55.3, 0.43, 1100)); // goal
    powerData.add(new CannonPowerData(62.1, 0.44, 1140));
    powerData.add(new CannonPowerData(69.5, 0.44, 1140));
    powerData.add(new CannonPowerData(77.2, 0.45, 1180));
    powerData.add(new CannonPowerData(85.0, 0.45, 1180));
    powerData.add(new CannonPowerData(92.4, 0.48, 1260));
    powerData.add(new CannonPowerData(101.7, 0.50, 1280));
    powerData.add(new CannonPowerData(113.7, 0.55, 1360)); // audience
  }

  public void fire(CannonPowerData cpd, ObeliskTagEnum obeliskTag) {
    if (cannonEnabled) {
      if (obeliskTag == ObeliskTagEnum.GPP) {
        fireLeft(cpd, false);
        fireRight(cpd, true);
        fireRight(cpd, false);
      } else if (obeliskTag == ObeliskTagEnum.PGP) {
        fireRight(cpd, false);
        fireLeft(cpd, false);
        fireRight(cpd, false);
      } else {
        fireRight(cpd, false);
        fireRight(cpd, true);
        fireLeft(cpd, false);
      }

      idle();
    }
  }

  public void fireLeft(CannonPowerData cpd, boolean waitTillDown) {
    setPower(cpd);
    leftTriggerServo.setPosition(LEFT_TRIGGER_UP_POSITION);
    FtcUtils.sleep(TRIGGER_MOVE_TIME);
    leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
    if (waitTillDown) {
      FtcUtils.sleep(TRIGGER_MOVE_TIME);
    }
  }

  public void fireRight(CannonPowerData cpd, boolean waitTillDown) {
    setPower(cpd);
    rightTriggerServo.setPosition(RIGHT_TRIGGER_UP_POSITION);
    FtcUtils.sleep(TRIGGER_MOVE_TIME);
    rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
    if (waitTillDown) {
      FtcUtils.sleep(TRIGGER_MOVE_TIME);
    }
  }

  /**
   * Finds the closest matching power for the given distance.
   *
   * @param distance The distance to the goal april tag.
   * @return Power for the cannon motors.
   */
  private CannonPowerData getClosestData(double distance) {
    CannonPowerData closestData = powerData.get(0);
    double minDifference = Math.abs(distance - closestData.distance);
    for (int i = 1; i < powerData.size(); i++) {
      double currentDifference = Math.abs(distance - powerData.get(i).distance);
      if (currentDifference < minDifference) {
        closestData = powerData.get(i);
      }
    }

    return closestData;
  }

  /**
   * Calculate PID power to ramp the cannon up to the given cannon power data.
   *
   * @param cpd The cannon power data to use.
   */
  private double getPidPower(CannonPowerData cpd) {
    // PID controller with a floor power value.
    // Floor power is the final power required.
    // P (Gain) is 0 for non positive feedback.
    // From velocity vs power data:
    // slope (v/p)= (2680-40)/(1.00-0.04)=2750
    double power = cpd.power + Math.max(0, (cpd.velocity - leftCannonMotor.getVelocity()) / 2750.0);
    power = Range.clip(power, FtcMotor.ZERO_POWER, FtcMotor.MAX_POWER);
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
    if (cannonEnabled) {
      if (leftCannonMotor != null && rightCannonMotor != null) {
        double distance = 0.0;
        if (gamePad1.y || gamePad2.y) {
          distance = powerData.get(1).distance;
        } else if (gamePad1.a || gamePad2.a) {
          distance = powerData.get(powerData.size() - 1).distance;
        } else if (gamePad1.right_trigger >= 0.5 || gamePad2.right_trigger >= 0.5) {
          if (parent != null && parent.aprilTag != null) {
            distance = parent.aprilTag.getGoalRange();
          }
        }

        setPower(getPidPower(getClosestData(distance)));
      }

      if (leftTriggerServo != null) {
        if (gamePad1.xWasPressed() || gamePad2.xWasPressed()) {
          leftTriggerServo.setPosition(LEFT_TRIGGER_UP_POSITION);
        } else if (gamePad1.xWasReleased() || gamePad2.xWasReleased()) {
          leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
        }
      }

      if (rightTriggerServo != null) {
        if (gamePad1.bWasPressed() || gamePad2.bWasPressed()) {
          rightTriggerServo.setPosition(RIGHT_TRIGGER_UP_POSITION);
        } else if (gamePad1.bWasReleased() || gamePad2.bWasReleased()) {
          rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
        }
      }
    }
  }

  /**
   * Sets cannon power. Doesn't wait for cannon to actually ramp up.
   *
   * @param power The motor power to set for both cannons.
   */
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
   * Sets cannon power. Waits for cannon to ramp up and be ready to fire.
   *
   * @param cpd The cannon power data to use.
   */
  public void setPower(CannonPowerData cpd) {
    if (cannonEnabled) {
      Deadline d = new Deadline(CANNON_RAMP_UP_TIME, TimeUnit.MILLISECONDS);
      while (!d.hasExpired() && leftCannonMotor.getVelocity() < cpd.velocity) {
        double power = getPidPower(cpd);
        setPower(power);
        FtcUtils.sleep(10);
      }
    }
  }

  /**
   * Displays cannon telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (cannonEnabled && telemetryEnabled) {
      if (leftCannonMotor != null && rightCannonMotor != null) {
        telemetry.addData(TAG, String.format(Locale.US, "Left: %.2f (%4.0f), Right: %.2f (%.0f)",
            leftCannonMotor.getPower(), leftCannonMotor.getVelocity(),
            rightCannonMotor.getPower(), rightCannonMotor.getVelocity()));
      }

      if (leftTriggerServo != null && rightTriggerServo != null) {
        telemetry.addData(TAG, String.format(Locale.US, "Left: %s, Right: %s",
            leftTriggerServo.getPosition() == LEFT_TRIGGER_UP_POSITION ? "Up" : "Down",
            rightTriggerServo.getPosition() == RIGHT_TRIGGER_UP_POSITION ? "Up" : "Down"));
      }
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (cannonEnabled) {
      idle();

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
      setPower(FtcMotor.ZERO_POWER);
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
