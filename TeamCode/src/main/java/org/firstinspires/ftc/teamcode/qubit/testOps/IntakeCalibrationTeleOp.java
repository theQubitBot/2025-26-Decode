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

@Disabled
@TeleOp(group = "TestOp")
public class IntakeCalibrationTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;

  FtcServo leftSpinServo = null;
  FtcServo rightSpinServo = null;
  FtcServo verticalSpinServo = null;
  double leftSpinPower, rightSpinPower, verticalSpinPower;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();

    leftSpinServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.LEFT_SPIN_SERVO_NAME));
    if (leftSpinServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      leftSpinServo.getController().pwmEnable();
    }

    leftSpinServo.setDirection(Servo.Direction.REVERSE);
    leftSpinPower = FtcIntake.SPIN_STOP_POWER;
    leftSpinServo.setPosition(leftSpinPower);

    rightSpinServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.RIGHT_SPIN_SERVO_NAME));
    if (rightSpinServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      rightSpinServo.getController().pwmEnable();
    }

    rightSpinServo.setDirection(Servo.Direction.FORWARD);
    rightSpinPower = FtcIntake.SPIN_STOP_POWER;
    rightSpinServo.setPosition(rightSpinPower);

    verticalSpinServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.VERTICAL_SPIN_SERVO_NAME));
    if (verticalSpinServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      verticalSpinServo.getController().pwmEnable();
    }

    verticalSpinServo.setDirection(Servo.Direction.FORWARD);
    verticalSpinPower = FtcIntake.SPIN_STOP_POWER;
    verticalSpinServo.setPosition(verticalSpinPower);

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
      leftSpinPower = Math.abs(gamepad1.left_stick_y);
      position = leftSpinPower;
      servo = leftSpinServo;
    } else {
      leftSpinPower = FtcServo.MID_POSITION;
      leftSpinServo.setPosition(leftSpinPower);
    }

    if (gamepad1.right_stick_y >= 0.5 || gamepad1.right_stick_y <= -0.5) {
      rightSpinPower = Math.abs(gamepad1.right_stick_y);
      position = rightSpinPower;
      servo = rightSpinServo;
    } else {
      rightSpinPower = FtcServo.MID_POSITION;
      rightSpinServo.setPosition(rightSpinPower);
    }

    if (gamepad1.dpad_up) {
      verticalSpinPower = FtcIntake.VERTICAL_SPIN_IN_POWER;
      position = verticalSpinPower;
      servo = verticalSpinServo;
    } else {
      verticalSpinPower = FtcServo.MID_POSITION;
      verticalSpinServo.setPosition(verticalSpinPower);
    }

    if (servo != null) {
      position = Range.clip(position, Servo.MIN_POSITION, Servo.MAX_POSITION);
      servo.setPosition(position);
    }

    telemetry.addData("Right spin", "right stick Y");
    telemetry.addData("Left spin", "left stick Y");
    telemetry.addData("Vertical spin", "dPad up");
    telemetry.addData("Right flip", "right trigger/bumper");
    telemetry.addData("Left flip", "left trigger/bumper");
    telemetry.addLine();
    telemetry.addData("Position", "LSpin %5.4f RSpin %5.4f",
        leftSpinPower, rightSpinPower);
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
