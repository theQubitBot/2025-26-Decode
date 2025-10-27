package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

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
  public static final double LEFT_CANNON_ZERO_VELOCITY = 0;
  public static final double RIGHT_CANNON_ZERO_VELOCITY = LEFT_CANNON_ZERO_VELOCITY;
  public static final double LEFT_CANNON_IDLE_VELOCITY = 100;
  public static final double RIGHT_CANNON_IDLE_VELOCITY = LEFT_CANNON_IDLE_VELOCITY;
  public static final double RIGHT_CANNON_IDLE_POWER = LEFT_CANNON_IDLE_POWER;
  public static final long CANNON_RAMP_UP_TIME = 4000; // milliseconds
  public static final String LEFT_TRIGGER_SERVO_NAME = "leftTriggerServo";
  public static final String RIGHT_TRIGGER_SERVO_NAME = "rightTriggerServo";
  public static final double LEFT_TRIGGER_UP_POSITION = 0.5745;
  public static final double LEFT_TRIGGER_DOWN_POSITION = 0.5050;
  public static final double RIGHT_TRIGGER_UP_POSITION = 0.4365;
  public static final double RIGHT_TRIGGER_DOWN_POSITION = 0.5020;
  public static final int TRIGGER_MOVE_TIME = 1000; // milliseconds
  public static final double FIRING_VELOCITY_MARGIN = 20;
  private final boolean cannonEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  private final FtcBot parent;
  public FtcMotor leftCannonMotor = null, rightCannonMotor = null;
  public FtcServo leftTriggerServo = null, rightTriggerServo = null;

  public List<CannonControlData> controlData = new ArrayList<>(10);

  /* Constructor */
  public FtcCannon(FtcBot robot) {
    parent = robot;
    // camera distance, motor power, motor velocity
    controlData.add(new CannonControlData(0, LEFT_CANNON_IDLE_VELOCITY)); // idle
    controlData.add(new CannonControlData(46.7, 1240)); // goal auto
    controlData.add(new CannonControlData(51.4, 1260)); // g
    controlData.add(new CannonControlData(56.4, 1260)); //g
    controlData.add(new CannonControlData(61.6, 1280)); //g
    controlData.add(new CannonControlData(66.7, 1280)); // g
    controlData.add(new CannonControlData(71.9, 1300)); // g
    controlData.add(new CannonControlData(108.0, 1320)); // audience close side
    controlData.add(new CannonControlData(115.0, 1400)); // audience mid side
    controlData.add(new CannonControlData(125.0, 1400)); // audience far side
  }

  public void fire(CannonControlData ccd, ObeliskTagEnum obeliskTag, LinearOpMode autoOpMode) {
    if (cannonEnabled) {
      if (obeliskTag == ObeliskTagEnum.GPP) {
        fireLeft(ccd, false, autoOpMode);
        fireRight(ccd, false, autoOpMode);
        fireRight(ccd, false, autoOpMode);
      } else if (obeliskTag == ObeliskTagEnum.PGP) {
        fireRight(ccd, false, autoOpMode);
        fireLeft(ccd, false, autoOpMode);
        fireRight(ccd, false, autoOpMode);
      } else {
        fireRight(ccd, false, autoOpMode);
        fireRight(ccd, false, autoOpMode);
        fireLeft(ccd, false, autoOpMode);
      }

      idle();
    }
  }

  public void fireLeft(CannonControlData ccd, boolean waitTillDown,LinearOpMode autoOpMode) {
    setVelocity(ccd, true);
    leftTriggerServo.setPosition(LEFT_TRIGGER_UP_POSITION);
    FtcUtils.interruptedSleep(TRIGGER_MOVE_TIME, autoOpMode);
    leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
    if (waitTillDown) {
      FtcUtils.interruptedSleep(TRIGGER_MOVE_TIME, autoOpMode);
    }
  }

  public void fireRight(CannonControlData ccd, boolean waitTillDown, LinearOpMode autoOpMode) {
    setVelocity(ccd, true);
    rightTriggerServo.setPosition(RIGHT_TRIGGER_UP_POSITION);
    FtcUtils.interruptedSleep(TRIGGER_MOVE_TIME, autoOpMode);
    rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
    if (waitTillDown) {
      FtcUtils.interruptedSleep(TRIGGER_MOVE_TIME, autoOpMode);
    }
  }

  /**
   * Finds the closest matching power for the given distance.
   *
   * @param distance The distance to the goal april tag.
   * @return Power for the cannon motors.
   */
  public CannonControlData getClosestData(double distance) {
    CannonControlData closestData = controlData.get(0);
    double minDifference = Math.abs(distance - closestData.distance);
    for (int i = 1; i < controlData.size(); i++) {
      double currentDifference = Math.abs(distance - controlData.get(i).distance);
      if (currentDifference < minDifference) {
        closestData = controlData.get(i);
      }
    }

    return closestData;
  }

  /**
   * Returns the current cannon power.
   */
  public double getPower() {
    double power = FtcMotor.ZERO_POWER;
    if (cannonEnabled) {
      if (leftCannonMotor != null) {
        power = (leftCannonMotor.getPower() + rightCannonMotor.getPower()) / 2.0;
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
      leftCannonMotor.setVelocity(LEFT_CANNON_IDLE_VELOCITY);
      rightCannonMotor.setVelocity(RIGHT_CANNON_IDLE_VELOCITY);
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
      leftCannonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

      rightCannonMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, RIGHT_CANNON_MOTOR_NAME));
      rightCannonMotor.setDirection(DcMotorEx.Direction.REVERSE);
      rightCannonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      rightCannonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
        CannonControlData ccd = getClosestData(0);
        if (gamePad1.y || gamePad2.y) {
          ccd = getClosestData(46);
        } else if (gamePad1.a || gamePad2.a) {
          ccd = getClosestData(115);
        } else if (gamePad1.right_trigger >= 0.5 || gamePad2.right_trigger >= 0.5) {
          if (parent != null && parent.aprilTag != null) {
            ccd = getClosestData(parent.aprilTag.getGoalRange());
          }
        }

        if (ccd.velocity == LEFT_CANNON_IDLE_VELOCITY) {
          // Factor 0.5, 0.8, 09 - lead to negative velocity
          // Delta 20, 40, 60, 80, 100
          double velocity = getVelocity() - 20;
          velocity = Math.max(velocity, LEFT_CANNON_IDLE_VELOCITY);
          setVelocity(velocity, false);
        } else {
          setVelocity(ccd.velocity, false);
        }
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

  public void setVelocity(CannonControlData ccd, boolean waitTillCompletion) {
    setVelocity(ccd.velocity, waitTillCompletion);
  }

  public void setVelocity(double velocity, boolean waitTillCompletion) {
    if (cannonEnabled) {
      leftCannonMotor.setVelocity(velocity);
      rightCannonMotor.setVelocity(velocity);
      if (waitTillCompletion) {
        Deadline d = new Deadline(CANNON_RAMP_UP_TIME, TimeUnit.MILLISECONDS);
        while (!d.hasExpired() && !FtcUtils.areEqual(getVelocity(), velocity, FIRING_VELOCITY_MARGIN)) {
          FtcUtils.sleep(FtcUtils.CYCLE_MS);
        }
      }
    }
  }

  /*
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
      setVelocity(LEFT_CANNON_ZERO_VELOCITY, false);
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
