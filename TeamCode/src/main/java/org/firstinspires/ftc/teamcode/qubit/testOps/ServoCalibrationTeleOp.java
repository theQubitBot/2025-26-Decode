package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcArm;
import org.firstinspires.ftc.teamcode.qubit.core.FtcCatapult;
import org.firstinspires.ftc.teamcode.qubit.core.FtcHand;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcServo;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

@Disabled
@TeleOp(group = "TestOp")
public class ServoCalibrationTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;
  static final int CYCLE_MS = 50;           // period of each cycle
  static final String SERVO_NAME = FtcCatapult.CATAPULT_SERVO_NAME;

  Servo handServo;
  double handPosition;
  Servo deliveryServo;
  double deliveryPosition;
  Servo armServo;
  double armPosition;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(">", "Initializing, please wait...");
    telemetry.update();
    handServo = hardwareMap.get(Servo.class, FtcHand.HAND_SERVO_NAME);
    deliveryServo = hardwareMap.get(Servo.class, FtcHand.DELIVERY_SERVO_NAME);
    armServo = hardwareMap.get(Servo.class, FtcArm.ARM_SERVO_NAME);
    handPosition = FtcHand.HAND_OPEN_POSITION;
    deliveryPosition = FtcHand.HAND_DOWN_POSITION;
    armPosition = FtcArm.ARM_FORWARD_POSITION;
    handServo.setPosition(handPosition);
    deliveryServo.setPosition(deliveryPosition);
    armServo.setPosition(armPosition);
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
    double position = 0;
    Servo servo = null;

    if (gamepad1.dpad_up) {
      deliveryPosition += FtcServo.LARGE_INCREMENT;
      servo = deliveryServo;
      position = deliveryPosition;
    } else if (gamepad1.dpad_down) {
      deliveryPosition -= FtcServo.LARGE_INCREMENT;
      servo = deliveryServo;
      position = deliveryPosition;
    } else if (gamepad1.dpad_left) {
      handPosition += FtcServo.LARGE_INCREMENT;
      servo = handServo;
      position = handPosition;
    } else if (gamepad1.dpad_right) {
      handPosition -= FtcServo.LARGE_INCREMENT;
      servo = handServo;
      position = handPosition;
    } else if (gamepad1.right_trigger > 0.5) {
      armPosition += FtcServo.LARGE_INCREMENT;
      servo = armServo;
      position = armPosition;
    } else if (gamepad1.right_bumper) {
      armPosition -= FtcServo.LARGE_INCREMENT;
      servo = armServo;
      position = armPosition;
    }

    if (servo != null) {
      position = Range.clip(position, Servo.MIN_POSITION, Servo.MAX_POSITION);
      servo.setPosition(position);
    }

    telemetry.addData("hand delivery", "dPad up/down");
    telemetry.addData("hand fingers", "dPad left/right");
    telemetry.addData("arm", "right trigger/bumper");
    telemetry.addData("Position", "hand %5.4f wrist %5.4f arm %5.4f",
        handPosition, deliveryPosition, armPosition);
    telemetry.addData(">", "Loop %.0f ms, cumulative %.0f seconds",
        loopTime.milliseconds(), runtime.seconds());
    telemetry.update();
    FtcUtils.sleep(CYCLE_MS);
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    telemetry.addData(">", "Tele Op stopped.");
    telemetry.update();
    FtcLogger.exit();
  }
}
