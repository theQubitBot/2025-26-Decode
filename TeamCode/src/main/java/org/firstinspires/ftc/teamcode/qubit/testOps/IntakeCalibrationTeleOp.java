package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcIntake;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcServo;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

//@Disabled
@TeleOp(group = "TestOp")
public class IntakeCalibrationTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;

  FtcServo leftRollerServo = null;
  FtcServo rightRollerServo = null;
  FtcServo leftSweeperServo = null;
  FtcServo rightSweeperServo = null;
  double leftRollerPower, rightRollerPower, leftSweeperPower, rightSweeperPower;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();

    leftRollerServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.LEFT_ROLLER_SERVO_NAME));
    if (leftRollerServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      leftRollerServo.getController().pwmEnable();
    }

    leftRollerServo.setDirection(Servo.Direction.REVERSE);
    leftRollerPower = FtcIntake.INTAKE_STOP_POWER;
    leftRollerServo.setPosition(leftRollerPower);

    rightRollerServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.RIGHT_ROLLER_SERVO_NAME));
    if (rightRollerServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      rightRollerServo.getController().pwmEnable();
    }

    rightRollerServo.setDirection(Servo.Direction.FORWARD);
    rightRollerPower = FtcIntake.INTAKE_STOP_POWER;
    rightRollerServo.setPosition(rightRollerPower);

    leftSweeperServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.LEFT_SWEEPER_SERVO_NAME));
    if (leftSweeperServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      leftSweeperServo.getController().pwmEnable();
    }

    leftSweeperServo.setDirection(Servo.Direction.FORWARD);
    leftSweeperPower = FtcIntake.INTAKE_STOP_POWER;
    leftSweeperServo.setPosition(leftSweeperPower);

    rightSweeperServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.RIGHT_SWEEPER_SERVO_NAME));
    if (rightSweeperServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      rightSweeperServo.getController().pwmEnable();
    }

    rightSweeperServo.setDirection(Servo.Direction.REVERSE);
    rightSweeperPower = FtcIntake.INTAKE_STOP_POWER;
    rightSweeperServo.setPosition(rightSweeperPower);

    FtcLogger.exit();
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {
    telemetry.addData(FtcUtils.TAG, "Waiting for driver to press play");
    telemetry.update();
    FtcUtils.sleep(FtcUtils.CYCLE_MS);
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Starting...");
    telemetry.update();
    runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    FtcLogger.exit();
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {
    FtcLogger.enter();
    // Show the elapsed game time and wheel power.
    loopTime.reset();
    FtcServo servo = null;
    double position = FtcServo.MID_POSITION;

    if (gamepad1.left_stick_y >= 0.5 || gamepad1.left_stick_y <= -0.5) {
      leftSweeperPower = Math.abs(gamepad1.left_stick_y);
      position = leftSweeperPower;
      servo = leftSweeperServo;
    } else {
      leftSweeperPower = FtcServo.MID_POSITION;
      leftSweeperServo.setPosition(leftSweeperPower);
    }

    if (gamepad1.right_stick_y >= 0.5 || gamepad1.right_stick_y <= -0.5) {
      rightSweeperPower = Math.abs(gamepad1.right_stick_y);
      position = rightSweeperPower;
      servo = rightSweeperServo;
    } else {
      rightSweeperPower = FtcServo.MID_POSITION;
      rightSweeperServo.setPosition(rightSweeperPower);
    }

    if (gamepad1.dpad_left) {
      leftRollerPower = FtcIntake.ROLLER_IN_POWER;
      position = leftRollerPower;
      servo = leftRollerServo;
    } else {
      leftRollerPower = FtcServo.MID_POSITION;
      leftRollerServo.setPosition(leftRollerPower);
    }

    if (gamepad1.dpad_right) {
      rightRollerPower = FtcIntake.ROLLER_IN_POWER;
      position = rightRollerPower;
      servo = rightRollerServo;
    } else {
      rightRollerPower = FtcServo.MID_POSITION;
      rightRollerServo.setPosition(rightRollerPower);
    }

    if (servo != null) {
      position = Range.clip(position, Servo.MIN_POSITION, Servo.MAX_POSITION);
      servo.setPosition(position);
    }

    telemetry.addData("Left sweeper", "left stick Y");
    telemetry.addData("Right sweeper", "right stick Y");
    telemetry.addData("Left roller", "dPad left");
    telemetry.addData("Right roller", "dPad right");
    telemetry.addLine();
    telemetry.addData("Position", "LSweeper %5.4f RSweeper %5.4f",
        leftSweeperPower, rightSweeperPower);
    telemetry.addData("Position", "LRoller %5.4f RRoller %5.4f",
        leftRollerPower, rightRollerPower);
    telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds",
        loopTime.milliseconds(), runtime.seconds());
    telemetry.update();
    FtcUtils.sleep(FtcUtils.CYCLE_MS);
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
    telemetry.update();
    FtcLogger.exit();
  }
}
