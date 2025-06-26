package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcHand;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcServo;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

@Disabled
@TeleOp(group = "TestOp")
public class HandCalibrationTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;

  FtcHand ftcHand = null;
  double handServoPosition;
  double deliveryServoPosition;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(">", "Initializing, please wait...");
    telemetry.update();
    ftcHand = new FtcHand();
    ftcHand.init(hardwareMap, telemetry, null);

    handServoPosition = FtcServo.MID_POSITION;
    deliveryServoPosition = FtcServo.MID_POSITION;
    telemetry.update();
    FtcLogger.exit();
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {
    telemetry.addData(">", "Waiting for driver to press play");
    telemetry.update();
    FtcUtils.sleep(50);
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    FtcLogger.enter();
    telemetry.addData(">", "Starting...");
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
    telemetry.addData(">", "Use left/right stick to small adjust left/right servo position");

    if (gamepad1.left_stick_x > 0.5) {
      handServoPosition += FtcServo.SMALL_INCREMENT;
    } else if (gamepad1.left_stick_x < -0.5) {
      handServoPosition -= FtcServo.SMALL_INCREMENT;
    }

    handServoPosition = Range.clip(handServoPosition, Servo.MIN_POSITION, Servo.MAX_POSITION);
    ftcHand.handServo.setPosition(handServoPosition);

    if (gamepad1.right_stick_x > 0.5) {
      deliveryServoPosition += FtcServo.SMALL_INCREMENT;
    } else if (gamepad1.right_stick_x < -0.5) {
      deliveryServoPosition -= FtcServo.SMALL_INCREMENT;
    }

    deliveryServoPosition = Range.clip(deliveryServoPosition, Servo.MIN_POSITION, Servo.MAX_POSITION);
    ftcHand.deliveryServo.setPosition(deliveryServoPosition);

    ftcHand.showTelemetry();
    telemetry.addData(">", "Loop %.0f ms, cumulative %.0f seconds",
        loopTime.milliseconds(), runtime.seconds());
    telemetry.update();
    FtcUtils.sleep(10);
    FtcLogger.exit();
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    ftcHand.stop();
    telemetry.addData(">", "Tele Op stopped.");
    telemetry.update();
    FtcLogger.exit();
  }
}
