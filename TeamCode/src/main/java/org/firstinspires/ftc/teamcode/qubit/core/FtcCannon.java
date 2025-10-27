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
  public static final long CANNON_RAMP_UP_TIME = 5000; // milliseconds
  public static final String LEFT_TRIGGER_SERVO_NAME = "leftTriggerServo";
  public static final String RIGHT_TRIGGER_SERVO_NAME = "rightTriggerServo";
  public static final double LEFT_TRIGGER_UP_POSITION = 0.5745;
  public static final double LEFT_TRIGGER_DOWN_POSITION = 0.5050;
  public static final double RIGHT_TRIGGER_UP_POSITION = 0.4365;
  public static final double RIGHT_TRIGGER_DOWN_POSITION = 0.5020;
  public static final int TRIGGER_MOVE_TIME = 1000; // milliseconds
  public static final double FIRING_VELOCITY_MARGIN = 40;
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
    // camera distance, motor power, motor velocity
    powerData.add(new CannonPowerData(30, 0.05, 100)); // idle
    powerData.add(new CannonPowerData(42.0, 0.44, 1120));
    powerData.add(new CannonPowerData(46.7, 0.45, 1140));
    powerData.add(new CannonPowerData(51.4, 0.47, 1180));
    powerData.add(new CannonPowerData(56.4, 0.48, 1200));
    powerData.add(new CannonPowerData(61.6, 0.49, 1220));
    powerData.add(new CannonPowerData(66.7, 0.50, 1220));
    powerData.add(new CannonPowerData(71.9, 0.51, 1240));
    powerData.add(new CannonPowerData(77.5, 0.52, 1240));
    powerData.add(new CannonPowerData(82.3, 0.53, 1260));
    powerData.add(new CannonPowerData(87.5, 0.54, 1320));
    powerData.add(new CannonPowerData(92.3, 0.55, 1380));
    powerData.add(new CannonPowerData(97.7, 0.56, 1380));
    powerData.add(new CannonPowerData(102.8, 0.58, 1400));
    powerData.add(new CannonPowerData(108.0, 0.59, 1420));
    powerData.add(new CannonPowerData(113.3, 0.60, 1420));
    powerData.add(new CannonPowerData(118.7, 0.62, 1440));
    powerData.add(new CannonPowerData(124.5, 0.64, 1460));
    powerData.add(new CannonPowerData(129.3, 0.70, 1520)); // audience
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
  public CannonPowerData getClosestData(double distance) {
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
    // P (Gain) is proportional to velocity difference.
    // From velocity vs power data:
    // slope (v/p)= (1460-1040)/(0.64-0.43)=2000
    double power = cpd.power + Math.pow(cpd.velocity - getVelocity() / 2000.0, 2.0);
    power=cpd.power;
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

  /**
   * Returns the current cannon velocity.
   */
  public double getVelocity() {
    double velocity = 0;
    if (cannonEnabled) {
      if (leftCannonMotor != null) {
        velocity = leftCannonMotor.getVelocity();
      }
    }

    return velocity;
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
          distance = getClosestData(42).distance;
        } else if (gamePad1.a || gamePad2.a) {
          distance = getClosestData(107.3).distance;
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
      while (!d.hasExpired() && !FtcUtils.areEqual(getVelocity(), cpd.velocity, FIRING_VELOCITY_MARGIN)) {
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
