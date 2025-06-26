package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcArm;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcServo;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

@Disabled
@TeleOp(group = "TestOp")
public class ControlledSpeedServoTeleOp extends OpMode {

  private static final long MOVE_TIME_MIN = 500;
  private static final long MOVE_TIME_MAX = 5000;
  private static final long MOVE_TIME_STEP = 100;
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;
  private long moveTime = 0;

  FtcServo servo;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(">", "Initializing, please wait...");
    telemetry.update();
    servo = new FtcServo(hardwareMap.get(Servo.class, FtcArm.ARM_SERVO_NAME));
    servo.getController().pwmEnable();
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
    servo.setPosition(FtcArm.ARM_FORWARD_POSITION);
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

    if (gamepad1.dpad_up) {
      moveTime += MOVE_TIME_STEP;
    } else if (gamepad1.dpad_down) {
      moveTime -= MOVE_TIME_STEP;
    }

    moveTime = (long) Range.clip(moveTime, MOVE_TIME_MIN, MOVE_TIME_MAX);
    if (gamepad1.right_bumper) {
      servo.controlledMove(FtcArm.ARM_BACKWARD_POSITION);
    } else if (gamepad1.right_trigger > 0.5) {
      servo.setPosition(FtcArm.ARM_FORWARD_POSITION);
    }

    telemetry.addData(">", "moveTime %d", moveTime);
    telemetry.addData(">", "Loop %.0f ms, cumulative %.0f seconds",
        loopTime.milliseconds(), runtime.seconds());
    telemetry.update();
    FtcUtils.sleep(500);
    FtcLogger.exit();
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
