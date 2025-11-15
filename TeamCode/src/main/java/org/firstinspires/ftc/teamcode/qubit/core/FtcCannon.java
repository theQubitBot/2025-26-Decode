package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

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
  public static final double CANNON_ZERO_VELOCITY = 0;
  public static final double CANNON_IDLE_VELOCITY = 400;
  public static final long CANNON_RAMP_UP_TIME = 800; // milliseconds
  public static final String LEFT_TRIGGER_SERVO_NAME = "leftTriggerServo";
  public static final String RIGHT_TRIGGER_SERVO_NAME = "rightTriggerServo";
  public static final double LEFT_TRIGGER_UP_POSITION = 0.5745;
  public static final double LEFT_TRIGGER_DOWN_POSITION = 0.5050;
  public static final double RIGHT_TRIGGER_UP_POSITION = 0.4365;
  public static final double RIGHT_TRIGGER_DOWN_POSITION = 0.5020;
  public static final int TRIGGER_MOVE_TIME = 800; // milliseconds
  public static double MIN_VELOCITY = CANNON_ZERO_VELOCITY;
  public static double MAX_VELOCITY = 1700.0;
  public static final double FIRING_VELOCITY_MARGIN = 25;
  private static final PIDFCoefficients CANNON_PIDF = new PIDFCoefficients(60, 0.25, 2, 12.5);

  private final boolean cannonEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  private final FtcBot parent;
  private VoltageSensor voltageSensor = null;
  public FtcMotor leftCannonMotor = null, rightCannonMotor = null;
  public FtcServo leftTriggerServo = null, rightTriggerServo = null;

  public List<CannonControlData> controlData = new ArrayList<>(10);

  /* Constructor */
  public FtcCannon(FtcBot robot) {
    parent = robot;
    // camera distance, motor power, motor velocity
    controlData.add(new CannonControlData(0, CANNON_IDLE_VELOCITY)); // idle
    controlData.add(new CannonControlData(29, 820));
    controlData.add(new CannonControlData(34, 820)); // goal auto
    controlData.add(new CannonControlData(44, 840)); // g
    controlData.add(new CannonControlData(49, 860)); //g
    controlData.add(new CannonControlData(54.2, 920)); //g
    controlData.add(new CannonControlData(59.1, 1000)); //g
    controlData.add(new CannonControlData(69.5, 1040)); //g
    controlData.add(new CannonControlData(74.5, 1080)); //g
    controlData.add(new CannonControlData(109, 1400)); //g
    controlData.add(new CannonControlData(113, 1420)); // audience side
    controlData.add(new CannonControlData(122.7, 1460)); //
  }

  public void coast() {
    if (cannonEnabled) {
      leftCannonMotor.setVelocity(CANNON_ZERO_VELOCITY);
      rightCannonMotor.setVelocity(CANNON_ZERO_VELOCITY);
    }
  }

  public void fire(CannonControlData ccd, ObeliskTagEnum obeliskTag, LinearOpMode autoOpMode) {
    if (cannonEnabled) {
      if (obeliskTag == ObeliskTagEnum.GPP) {
        fireLeft(ccd, false, autoOpMode);
        fireRight(ccd, true, autoOpMode);
        fireRight(ccd, false, autoOpMode);
      } else if (obeliskTag == ObeliskTagEnum.PGP) {
        fireRight(ccd, false, autoOpMode);
        fireLeft(ccd, false, autoOpMode);
        fireRight(ccd, false, autoOpMode);
      } else {
        fireRight(ccd, true, autoOpMode);
        fireRight(ccd, false, autoOpMode);
        fireLeft(ccd, false, autoOpMode);
      }

      coast();
    }
  }

  public void fireLeft(CannonControlData ccd, boolean waitTillDown, LinearOpMode autoOpMode) {
    setVelocity(ccd, true);
    leftTriggerServo.setPosition(LEFT_TRIGGER_UP_POSITION);
    FtcUtils.interruptibleSleep(TRIGGER_MOVE_TIME, autoOpMode);
    leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
    if (waitTillDown) {
      FtcUtils.interruptibleSleep(TRIGGER_MOVE_TIME, autoOpMode);
    }
  }

  public void fireRight(CannonControlData ccd, boolean waitTillDown, LinearOpMode autoOpMode) {
    setVelocity(ccd, true);
    rightTriggerServo.setPosition(RIGHT_TRIGGER_UP_POSITION);
    FtcUtils.interruptibleSleep(TRIGGER_MOVE_TIME, autoOpMode);
    rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
    if (waitTillDown) {
      FtcUtils.interruptibleSleep(TRIGGER_MOVE_TIME, autoOpMode);
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

  public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode runMode) {
    return leftCannonMotor.getPIDFCoefficients(runMode);
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
      leftCannonMotor.setVelocity(CANNON_IDLE_VELOCITY);
      rightCannonMotor.setVelocity(CANNON_IDLE_VELOCITY);
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
      voltageSensor = hardwareMap.voltageSensor.iterator().next();
      leftCannonMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, LEFT_CANNON_MOTOR_NAME));
      // RUE limits max motor speed to 85% by default
      // Raise that limit to 100%
      MotorConfigurationType motorConfigurationType = leftCannonMotor.getMotorType().clone();
      motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
      leftCannonMotor.setMotorType(motorConfigurationType);
      leftCannonMotor.setDirection(DcMotorEx.Direction.FORWARD);
      leftCannonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      leftCannonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

      rightCannonMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, RIGHT_CANNON_MOTOR_NAME));
      motorConfigurationType = rightCannonMotor.getMotorType().clone();
      motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
      rightCannonMotor.setMotorType(motorConfigurationType);
      rightCannonMotor.setDirection(DcMotorEx.Direction.REVERSE);
      rightCannonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      rightCannonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

      setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, CANNON_PIDF);

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
        if (gamePad1.right_trigger >= 0.5 || gamePad2.right_trigger >= 0.5) {
          ccd = getClosestData(44);
        } else if (gamePad1.right_bumper || gamePad2.right_bumper) {
          ccd = getClosestData(113);
        } else if (gamePad1.y || gamePad2.y) {
          if (parent != null && parent.aprilTag != null) {
            ccd = getClosestData(parent.aprilTag.getGoalRange());
          }
        }

        if (ccd.velocity == CANNON_IDLE_VELOCITY) {
          if (getVelocity() > CANNON_IDLE_VELOCITY) {
            // Coast down to IDLE velocity
            coast();
          } else {
            idle();
          }
        } else {
          setVelocity(ccd, false);
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

  public void setPIDFCoefficients(DcMotorEx.RunMode runMode, PIDFCoefficients pidf) {
    if (cannonEnabled) {
      if (leftCannonMotor != null) {
        leftCannonMotor.setPIDFCoefficients(runMode, new PIDFCoefficients(
            pidf.p, pidf.i, pidf.d, pidf.f * 12.0 / voltageSensor.getVoltage()));
      }

      if (rightCannonMotor != null) {
        rightCannonMotor.setPIDFCoefficients(runMode, new PIDFCoefficients(
            pidf.p, pidf.i, pidf.d, pidf.f * 12.0 / voltageSensor.getVoltage()));
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
          FtcUtils.sleep(FtcUtils.INTERRUPTIBLE_CYCLE_MS);
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
      setVelocity(CANNON_ZERO_VELOCITY, false);
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
