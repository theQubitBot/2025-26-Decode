package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcCatapult;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcServo;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

@Disabled
@TeleOp(group = "TestOp")
public class CatapultCalibrationTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;
  static final int CYCLE_MS = 50;           // period of each cycle
  Servo servo;
  double servoPosition;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(">", "Initializing, please wait...");
    telemetry.update();
    servo = hardwareMap.get(Servo.class, FtcCatapult.CATAPULT_SERVO_NAME);
    servo.getController().pwmEnable();
    servoPosition = FtcServo.MID_POSITION;
    servo.setPosition(servoPosition);
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
    loopTime.reset();

    if (gamepad1.dpad_up) {
      servoPosition += FtcServo.SMALL_INCREMENT;
    } else if (gamepad1.dpad_down) {
      servoPosition -= FtcServo.SMALL_INCREMENT;
    } else if (gamepad1.left_bumper) {
      servoPosition += FtcServo.LARGE_INCREMENT;
    } else if (gamepad1.left_trigger > 0.5) {
      servoPosition -= FtcServo.LARGE_INCREMENT;
    }

    servoPosition = Range.clip(servoPosition, Servo.MIN_POSITION, Servo.MAX_POSITION);
    servo.setPosition(servoPosition);

    telemetry.addData(">", "Use dPad up/down to calibrate servo");
    telemetry.addData("Position", "%5.4f", servoPosition);
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
    servo.getController().pwmDisable();
    telemetry.addData(">", "Tele Op stopped.");
    telemetry.update();
    FtcLogger.exit();
  }
}
