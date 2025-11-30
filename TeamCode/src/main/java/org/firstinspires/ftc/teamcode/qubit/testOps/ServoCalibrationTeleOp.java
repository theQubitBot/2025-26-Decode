package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcAprilTag;
import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcIntake;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcServo;
import org.firstinspires.ftc.teamcode.qubit.core.FtcSorter;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.StringUtils;

@Disabled
@TeleOp(group = "TestOp")
public class ServoCalibrationTeleOp extends OpMode {
  // Declare OpMode members
  ElapsedTime runtime = null;
  ElapsedTime loopTime = null;
  FtcAprilTag aprilTag;
  FtcServo aprilTagServo, sorterServo, lightServo, leftTriggerServo, rightTriggerServo;
  FtcServo currentServo;
  double servoPosition;
  String servoName;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();
    aprilTag = new FtcAprilTag(null);
    aprilTag.init(hardwareMap, telemetry);

    aprilTagServo = new FtcServo(hardwareMap.get(Servo.class, FtcAprilTag.APRIL_TAG_SERVO_NAME));
    sorterServo = new FtcServo(hardwareMap.get(Servo.class, FtcSorter.SORTER_SERVO_NAME));
    lightServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.LIGHT_SERVO_NAME));
    leftTriggerServo = new FtcServo(hardwareMap.get(Servo.class, FtcCannon.LEFT_TRIGGER_SERVO_NAME));
    rightTriggerServo = new FtcServo(hardwareMap.get(Servo.class, FtcCannon.RIGHT_TRIGGER_SERVO_NAME));

    if (aprilTagServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      aprilTagServo.getController().pwmEnable();
    }

    if (leftTriggerServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      leftTriggerServo.getController().pwmEnable();
    }

    if (rightTriggerServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      rightTriggerServo.getController().pwmEnable();
    }

    if (sorterServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      sorterServo.getController().pwmEnable();
    }

    if (lightServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      lightServo.getController().pwmEnable();
    }

    servoPosition = FtcServo.MID_POSITION;
    aprilTagServo.setPosition(servoPosition);
    leftTriggerServo.setPosition(servoPosition);
    rightTriggerServo.setPosition(servoPosition);
    sorterServo.setPosition(servoPosition);
    lightServo.setPosition(0);
    currentServo = aprilTagServo;
    servoName = FtcAprilTag.APRIL_TAG_SERVO_NAME;
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
    aprilTag.start();
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

    telemetry.addData(StringUtils.Triangle, "aprilTag servo");
    telemetry.addData(StringUtils.Square, "left trigger servo");
    telemetry.addData(StringUtils.Circle, "right trigger servo");
    telemetry.addData(StringUtils.Cross, "artifact sensor servo");
    telemetry.addData("right trigger", "light servo");
    telemetry.addLine();
    telemetry.addData("dPad up/down", "large +/-");
    telemetry.addData("dPad left/right", "small +/-");
    telemetry.addLine();

    if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
      currentServo = aprilTagServo;
      servoName = FtcAprilTag.APRIL_TAG_SERVO_NAME;
    } else if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
      currentServo = leftTriggerServo;
      servoName = FtcCannon.LEFT_TRIGGER_SERVO_NAME;
    } else if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
      currentServo = rightTriggerServo;
      servoName = FtcCannon.RIGHT_TRIGGER_SERVO_NAME;
    } else if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
      currentServo = sorterServo;
      servoName = FtcSorter.SORTER_SERVO_NAME;
    } else if (gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
      currentServo = lightServo;
      servoName = FtcIntake.LIGHT_SERVO_NAME;
    }

    if (gamepad1.dpad_up || gamepad2.dpad_up) {
      servoPosition = currentServo.getPosition() + FtcServo.LARGE_INCREMENT;
    } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
      servoPosition = currentServo.getPosition() + FtcServo.SMALL_INCREMENT;
    } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
      servoPosition = currentServo.getPosition() - FtcServo.LARGE_INCREMENT;
    } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
      servoPosition = currentServo.getPosition() - FtcServo.SMALL_INCREMENT;
    }

    if (currentServo != null) {
      servoPosition = Range.clip(servoPosition, Servo.MIN_POSITION, Servo.MAX_POSITION);
      currentServo.setPosition(servoPosition);
    }

    if (currentServo == aprilTagServo) {
      aprilTag.showTelemetry();
    }

    telemetry.addData("Servo", "%s %5.4f", servoName, servoPosition);
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
