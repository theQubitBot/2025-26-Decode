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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObeliskTagEnum;

import java.util.concurrent.TimeUnit;

/**
 * A class to manage the cannon.
 */
public class FtcCannon extends FtcSubSystemBase {
  private static final String TAG = "FtcCannon";
  public static final String LEFT_HOOD_SERVO_NAME = "leftHoodServo";
  public static final String RIGHT_HOOD_SERVO_NAME = "rightHoodServo";
  public static final String LEFT_CANNON_MOTOR_NAME = "leftCannonMotor";
  public static final String RIGHT_CANNON_MOTOR_NAME = "rightCannonMotor";
  private static final double CANNON_ZERO_VELOCITY = 0;
  public static final double CANNON_IDLE_VELOCITY = 400;
  public static final String LEFT_TRIGGER_SERVO_NAME = "leftTriggerServo";
  public static final String RIGHT_TRIGGER_SERVO_NAME = "rightTriggerServo";
  public static final double LEFT_TRIGGER_UP_POSITION = 0.8400;
  public static final double LEFT_TRIGGER_DOWN_POSITION = 0.5550;
  public static final double RIGHT_TRIGGER_UP_POSITION = 0.2600;
  public static final double RIGHT_TRIGGER_DOWN_POSITION = 0.5700;
  private static final int TRIGGER_MOVE_UP_TIME = 350; // milliseconds
  private static final int TRIGGER_MOVE_DOWN_TIME = 350; // milliseconds
  public static double MIN_VELOCITY = CANNON_ZERO_VELOCITY;
  public static double MAX_VELOCITY = 1700.0;
  public static final double FIRING_VELOCITY_MARGIN = 25;
  public static final double ACHIEVABLE_MAX_RPM_FRACTION = 1.0;
  private static final PIDFCoefficients CANNON_VELOCITY_PIDF = new PIDFCoefficients(60, 0.25, 2, 13.0);

  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  private final BaseBot parent;
  private VoltageSensor voltageSensor = null;
  public FtcMotor leftCannonMotor = null, rightCannonMotor = null;
  public FtcServo leftTriggerServo = null, rightTriggerServo = null;
  public FtcServo leftHoodServo = null, rightHoodServo = null;
  private Deadline sorterMoveDeadline;

  /* Constructor */
  public FtcCannon(BaseBot robot) {
    parent = robot;
    sorterMoveDeadline = new Deadline(FtcSorter.SORTER_SERVO_MOVE_TIME, TimeUnit.MILLISECONDS);
  }

  private void coast() {
    leftCannonMotor.setVelocity(CANNON_ZERO_VELOCITY);
    rightCannonMotor.setVelocity(CANNON_ZERO_VELOCITY);
  }

  public void fire(CannonControlData ccd, ObeliskTagEnum obeliskTag, LinearOpMode autoOpMode) {
    if (obeliskTag == ObeliskTagEnum.GPP) {
      parent.sorter.pushGreen(true);
      fireLeft(ccd, false, autoOpMode);
      parent.sorter.setStraight(false);
      fireRight(ccd, true, autoOpMode);
      parent.sorter.tapPurple(autoOpMode);
      fireRight(ccd, false, autoOpMode);

      // handle extra or mis-sorted artifact
      parent.sorter.pushGreen(true);
      fireLeft(ccd, false, autoOpMode);
    } else if (obeliskTag == ObeliskTagEnum.PGP) {
      parent.sorter.setStraight(false);
      fireRight(ccd, false, autoOpMode);
      parent.sorter.pushGreen(true);
      fireLeft(ccd, false, autoOpMode);
      parent.sorter.pushPurple(true);
      fireRight(ccd, false, autoOpMode);
      parent.sorter.pushGreen(true);
      fireLeft(ccd, false, autoOpMode);
    } else { // PPG
      parent.sorter.setStraight(false);
      fireRight(ccd, true, autoOpMode);
      parent.sorter.tapPurple(autoOpMode);
      fireRight(ccd, false, autoOpMode);
      parent.sorter.pushGreen(true);
      fireLeft(ccd, false, autoOpMode);
      parent.sorter.pushPurple(true);
      fireRight(ccd, false, autoOpMode);
    }

    parent.sorter.setStraight(false);
    coast();
  }

  private void fireLeft(CannonControlData ccd, boolean waitTillDown, LinearOpMode autoOpMode) {
    setVelocity(ccd, true);
    leftTriggerServo.setPosition(LEFT_TRIGGER_UP_POSITION);
    FtcUtils.interruptibleSleep(TRIGGER_MOVE_UP_TIME, autoOpMode);
    leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
    if (waitTillDown) {
      FtcUtils.interruptibleSleep(TRIGGER_MOVE_DOWN_TIME, autoOpMode);
    }
  }

  private void fireRight(CannonControlData ccd, boolean waitTillDown, LinearOpMode autoOpMode) {
    setVelocity(ccd, true);
    rightTriggerServo.setPosition(RIGHT_TRIGGER_UP_POSITION);
    FtcUtils.interruptibleSleep(TRIGGER_MOVE_UP_TIME, autoOpMode);
    rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
    if (waitTillDown) {
      FtcUtils.interruptibleSleep(TRIGGER_MOVE_DOWN_TIME, autoOpMode);
    }
  }

  private long getCannonRampUpTime(double startVelocity, double endVelocity) {
    /*
    Linear: t(v)=mv+b, m=2.631, b=--2236.84
    return (long) Math.abs((endVelocity - startVelocity) * 0.5);

    Exponential: t(v)=A(ekv−1), A=25.8, k=0.0029
    return (long) (25.8 * Math.exp(0.0029 * Math.abs((endVelocity - startVelocity) -1.0)));

    Simplified numeric form t(v)≈70+0.60v
	  */
    return (long) Math.abs((endVelocity - startVelocity) * 0.6);
  }

  public double getHoodPosition() {
    double hoodPosition = CannonControlData.HOOD_MAX_POSITION;
    if (leftHoodServo != null)
      hoodPosition = leftHoodServo.getPosition();

    return hoodPosition;
  }

  public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode runMode) {
    return leftCannonMotor.getPIDFCoefficients(runMode);
  }

  /**
   * Returns the current cannon power.
   */
  public double getPower() {
    double power = FtcMotor.ZERO_POWER;
    if (leftCannonMotor != null && rightCannonMotor != null) {
      power = (leftCannonMotor.getPower() + rightCannonMotor.getPower()) / 2.0;
    }

    return power;
  }

  /**
   * Returns the current cannon velocity.
   */
  public double getVelocity() {
    // Motors are hardware synchronized. Read only one motor.
    double velocity = 0;
    if (leftCannonMotor != null) {
      velocity = leftCannonMotor.getVelocity();
    }


    return velocity;
  }

  private void idle() {
    leftCannonMotor.setVelocity(CANNON_IDLE_VELOCITY);
    rightCannonMotor.setVelocity(CANNON_IDLE_VELOCITY);
  }

  /**
   * Initialize standard Hardware interfaces
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  @Override
  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    FtcLogger.enter();
    this.telemetry = telemetry;

    voltageSensor = hardwareMap.voltageSensor.iterator().next();
    leftCannonMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, LEFT_CANNON_MOTOR_NAME));
    // RUE limits max motor speed to 85% by default
    // Raise that limit to 100%
    MotorConfigurationType motorConfigLeft = leftCannonMotor.getMotorType().clone();
    motorConfigLeft.setAchieveableMaxRPMFraction(ACHIEVABLE_MAX_RPM_FRACTION);
    leftCannonMotor.setMotorType(motorConfigLeft);
    leftCannonMotor.setDirection(DcMotorEx.Direction.FORWARD);
    leftCannonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    leftCannonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    rightCannonMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, RIGHT_CANNON_MOTOR_NAME));
    MotorConfigurationType motorConfigRight = rightCannonMotor.getMotorType().clone();
    motorConfigRight.setAchieveableMaxRPMFraction(ACHIEVABLE_MAX_RPM_FRACTION);
    rightCannonMotor.setMotorType(motorConfigRight);
    rightCannonMotor.setDirection(DcMotorEx.Direction.REVERSE);
    rightCannonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    rightCannonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, CANNON_VELOCITY_PIDF);

    leftTriggerServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_TRIGGER_SERVO_NAME));
    leftTriggerServo.setDirection(Servo.Direction.FORWARD);

    rightTriggerServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_TRIGGER_SERVO_NAME));
    rightTriggerServo.setDirection(Servo.Direction.FORWARD);

    leftHoodServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_HOOD_SERVO_NAME));
    leftHoodServo.setDirection(Servo.Direction.REVERSE);

    rightHoodServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_HOOD_SERVO_NAME));
    rightHoodServo.setDirection(Servo.Direction.FORWARD);

    showTelemetry();
    telemetry.addData(TAG, "initialized");

    FtcLogger.exit();
  }

  public boolean isPrimed(double distance) {
    CannonControlData ccd = CannonControlData.getClosestData(distance);
    return FtcUtils.areEqual(ccd.velocity, getVelocity(), FtcCannon.FIRING_VELOCITY_MARGIN);
  }

  /**
   * Operates the cannon using the gamePads.
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */
  @Override
  public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
    if (leftCannonMotor != null && rightCannonMotor != null) {
      CannonControlData ccd = CannonControlData.getClosestData(CannonControlData.GOAL_MIN_DISTANCE);
      if (gamePad1.right_trigger >= 0.5 || gamePad2.right_trigger >= 0.5) {
        ccd = CannonControlData.getClosestData(CannonControlData.GOAL_SWEET_SPOT_DISTANCE);
      } else if (gamePad1.right_bumper || gamePad2.right_bumper) {
        ccd = CannonControlData.getClosestData(CannonControlData.AUDIENCE_DISTANCE);
      } else if (parent.localizer.robotInLaunchZone() &&
          parent.localizer.robotPointingAtGoal()) {
        ccd = CannonControlData.getClosestData(parent.localizer.getGoalDistance());
      }

      if (ccd.velocity == CANNON_IDLE_VELOCITY) {
        if (getVelocity() > CANNON_IDLE_VELOCITY) {
          // Coast down to IDLE velocity
          coast();
        } else {
          idle();
        }
      } else {
        setHoodPosition(ccd);
        setVelocity(ccd, false);
      }
    }

    if (leftTriggerServo != null) {
      if (gamePad1.xWasPressed() || gamePad2.xWasPressed()) {
        if (parent != null && parent.sorter != null) {
          parent.sorter.pushGreen(false);
          sorterMoveDeadline.reset();
        }

        leftTriggerServo.setPosition(LEFT_TRIGGER_UP_POSITION);
      } else if(gamePad1.x || gamePad2.x){
        if(sorterMoveDeadline.hasExpired()){
          if (parent != null && parent.sorter != null) {
            parent.sorter.setStraight(false);
          }
        }
      } else if (gamePad1.xWasReleased() || gamePad2.xWasReleased()) {
        if (parent != null && parent.sorter != null) {
          parent.sorter.setStraight(false);
          sorterMoveDeadline.expire();
        }

        leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
      }
    }

    if (rightTriggerServo != null) {
      if (gamePad1.bWasPressed() || gamePad2.bWasPressed()) {
        if (parent != null && parent.sorter != null) {
          parent.sorter.pushPurple(false);
          sorterMoveDeadline.reset();
        }

        rightTriggerServo.setPosition(RIGHT_TRIGGER_UP_POSITION);
      } else if(gamePad1.b || gamePad2.b){
        if(sorterMoveDeadline.hasExpired()){
          if (parent != null && parent.sorter != null) {
            parent.sorter.setStraight(false);
          }
        }
      } else if (gamePad1.bWasReleased() || gamePad2.bWasReleased()) {
        if (parent != null && parent.sorter != null) {
          parent.sorter.setStraight(false);
          sorterMoveDeadline.expire();
        }

        rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
      }
    }
  }

  public void setHoodPosition(CannonControlData ccd) {
    setHoodPosition(ccd.hoodPosition);
  }

  public void setHoodPosition(double position) {
    position = Range.clip(position,
        CannonControlData.HOOD_MIN_POSITION, CannonControlData.HOOD_MAX_POSITION);
    if (leftHoodServo != null)
      leftHoodServo.setPosition(position);

    if (rightHoodServo != null)
      rightHoodServo.setPosition(position);
  }

  private void setPIDFCoefficients(DcMotorEx.RunMode runMode, PIDFCoefficients pidf) {
    double batteryVoltage = Math.max(12, voltageSensor.getVoltage()); // guard against 0 voltage
    if (leftCannonMotor != null) {
      leftCannonMotor.setPIDFCoefficients(runMode, new PIDFCoefficients(
          pidf.p, pidf.i, pidf.d, pidf.f * 12.0 / batteryVoltage));
    }

    if (rightCannonMotor != null) {
      rightCannonMotor.setPIDFCoefficients(runMode, new PIDFCoefficients(
          pidf.p, pidf.i, pidf.d, pidf.f * 12.0 / batteryVoltage));
    }
  }

  /**
   * Sets cannon power. Doesn't wait for cannon to actually ramp up.
   *
   * @param power The motor power to set for both cannons.
   */
  private void setPower(double power) {
    if (leftCannonMotor != null) {
      leftCannonMotor.setPower(power);
    }

    if (rightCannonMotor != null) {
      rightCannonMotor.setPower(power);
    }
  }

  private void setVelocity(CannonControlData ccd, boolean waitTillCompletion) {
    setVelocity(ccd.velocity, waitTillCompletion);
  }

  public void setVelocity(double velocity, boolean waitTillCompletion) {
    leftCannonMotor.setVelocity(velocity);
    rightCannonMotor.setVelocity(velocity);
    if (waitTillCompletion) {
      double currentVelocity = getVelocity();
      Deadline d = new Deadline(getCannonRampUpTime(currentVelocity, velocity), TimeUnit.MILLISECONDS);
      while (!d.hasExpired() && !FtcUtils.areEqual(currentVelocity, velocity, FIRING_VELOCITY_MARGIN)) {
        FtcUtils.sleep(FtcUtils.INTERRUPTIBLE_CYCLE_MS);
        currentVelocity = getVelocity();
      }
    }
  }

  /*
   * Displays cannon telemetry. Helps with debugging.
   */
  @Override
  public void showTelemetry() {
    FtcLogger.enter();
    if (telemetryEnabled && telemetry != null) {
      if (leftCannonMotor != null && rightCannonMotor != null) {
        telemetry.addData(TAG, "LM %4.0f, RM %4.0f",
            leftCannonMotor.getVelocity(), rightCannonMotor.getVelocity());
      }

      if (leftTriggerServo != null && rightTriggerServo != null) {
        telemetry.addData(TAG, "LeftT: %s, RightT: %s",
            FtcUtils.areEqual(leftTriggerServo.getPosition(), LEFT_TRIGGER_UP_POSITION, FtcUtils.EPSILON4) ? "Up" : "Down",
            FtcUtils.areEqual(rightTriggerServo.getPosition(), RIGHT_TRIGGER_UP_POSITION, FtcUtils.EPSILON4) ? "Up" : "Down");
      }
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    FtcLogger.enter();
    if (leftTriggerServo != null) {
      if (leftTriggerServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        leftTriggerServo.getController().pwmEnable();
      }

      // Move the servo for correct initialization.
      leftTriggerServo.setPosition(LEFT_TRIGGER_UP_POSITION);
      FtcUtils.sleep(1);
      leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
    }

    if (rightTriggerServo != null) {
      if (rightTriggerServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        rightTriggerServo.getController().pwmEnable();
      }

      // Move the servo for correct initialization.
      rightTriggerServo.setPosition(RIGHT_TRIGGER_UP_POSITION);
      FtcUtils.sleep(1);
      rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
    }

    if (leftHoodServo != null) {
      if (leftHoodServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        leftHoodServo.getController().pwmEnable();
      }

      // Move the servo for correct initialization.
      leftHoodServo.setPosition(CannonControlData.HOOD_MIN_POSITION);
      FtcUtils.sleep(1);
      leftHoodServo.setPosition(CannonControlData.HOOD_MAX_POSITION);
    }

    if (rightHoodServo != null) {
      if (rightHoodServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        rightHoodServo.getController().pwmEnable();
      }

      // Move the servo for correct initialization.
      rightHoodServo.setPosition(CannonControlData.HOOD_MIN_POSITION);
      FtcUtils.sleep(1);
      rightHoodServo.setPosition(CannonControlData.HOOD_MAX_POSITION);
    }

    FtcLogger.exit();
  }

  /**
   * Stops the cannon.
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    setVelocity(CANNON_ZERO_VELOCITY, false);
    if (leftTriggerServo != null) {
      leftTriggerServo.setPosition(LEFT_TRIGGER_DOWN_POSITION);
    }

    if (rightTriggerServo != null) {
      rightTriggerServo.setPosition(RIGHT_TRIGGER_DOWN_POSITION);
    }

    if (leftHoodServo != null) {
      leftHoodServo.setPosition(CannonControlData.HOOD_REGULAR_POSITION);
    }

    if (rightHoodServo != null) {
      rightHoodServo.setPosition(CannonControlData.HOOD_REGULAR_POSITION);
    }

    FtcLogger.exit();
  }
}
