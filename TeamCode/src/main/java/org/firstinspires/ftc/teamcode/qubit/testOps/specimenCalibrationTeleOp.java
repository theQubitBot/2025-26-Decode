package org.firstinspires.ftc.teamcode.qubit.testOps;

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
public class specimenCalibrationTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;

  FtcServo leftSpecimenServo = null;
  FtcServo rightSpecimenServo = null;
  double leftSpecimenServoPosition, rightSpecimenServoPosition;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();

    leftSpecimenServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.LEFT_SPECIMEN_SERVO_NAME));
    if (leftSpecimenServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      leftSpecimenServo.getController().pwmEnable();
    }

    leftSpecimenServo.setDirection(Servo.Direction.FORWARD);
    leftSpecimenServoPosition = FtcServo.MID_POSITION;
    leftSpecimenServo.setPosition(leftSpecimenServoPosition);

    rightSpecimenServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.RIGHT_SPECIMEN_SERVO_NAME));
    if (rightSpecimenServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      rightSpecimenServo.getController().pwmEnable();
    }

    rightSpecimenServo.setDirection(Servo.Direction.FORWARD);
    rightSpecimenServoPosition = FtcServo.MID_POSITION;
    rightSpecimenServo.setPosition(rightSpecimenServoPosition);

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
    loopTime.reset();
    FtcServo servo = null;
    double position = FtcServo.MID_POSITION;

    if (gamepad1.left_trigger > 0.5) {
      leftSpecimenServoPosition -= FtcServo.LARGE_INCREMENT;
      position = leftSpecimenServoPosition;
      servo = leftSpecimenServo;
    } else if (gamepad1.left_bumper) {
      leftSpecimenServoPosition += FtcServo.LARGE_INCREMENT;
      position = leftSpecimenServoPosition;
      servo = leftSpecimenServo;
    }

    if (gamepad1.right_trigger > 0.5) {
      rightSpecimenServoPosition -= FtcServo.LARGE_INCREMENT;
      position = rightSpecimenServoPosition;
      servo = rightSpecimenServo;
    } else if (gamepad1.right_bumper) {
      rightSpecimenServoPosition += FtcServo.LARGE_INCREMENT;
      position = rightSpecimenServoPosition;
      servo = rightSpecimenServo;
    }

    if (servo != null) {
      position = Range.clip(position, Servo.MIN_POSITION, Servo.MAX_POSITION);
      servo.setPosition(position);
    }

    telemetry.addData("Left specimen", "left trigger/bumper");
    telemetry.addData("Right specimen", "right trigger/bumper");
    telemetry.addLine();
    telemetry.addData("Position", "Left %5.4f right %5.4f",
        leftSpecimenServoPosition, rightSpecimenServoPosition);
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
