package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcServo;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

@Disabled
@TeleOp(group = "TestOp")
public class HoodCalibrationTeleOp extends OpMode {
  // Declare OpMode members
  ElapsedTime runtime = null;
  ElapsedTime loopTime = null;
  FtcServo leftHoodServo, rightHoodServo;
  double servoPosition;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();

    leftHoodServo = new FtcServo(hardwareMap.get(Servo.class, FtcCannon.LEFT_HOOD_SERVO_NAME));
    leftHoodServo.setDirection(Servo.Direction.REVERSE);
    if (leftHoodServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      leftHoodServo.getController().pwmEnable();
    }

    rightHoodServo = new FtcServo(hardwareMap.get(Servo.class, FtcCannon.RIGHT_HOOD_SERVO_NAME));
    rightHoodServo.setDirection(Servo.Direction.FORWARD);
    if (rightHoodServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
      rightHoodServo.getController().pwmEnable();
    }

    servoPosition = FtcServo.MID_POSITION;
    leftHoodServo.setPosition(servoPosition);
    rightHoodServo.setPosition(servoPosition);
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

    telemetry.addData("dPad up/down", "large +/-");
    telemetry.addData("dPad left/right", "small +/-");
    telemetry.addLine();

    if (gamepad1.dpad_up || gamepad2.dpad_up) {
      servoPosition = leftHoodServo.getPosition() + FtcServo.LARGE_INCREMENT;
    } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
      servoPosition = leftHoodServo.getPosition() + FtcServo.SMALL_INCREMENT;
    } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
      servoPosition = leftHoodServo.getPosition() - FtcServo.LARGE_INCREMENT;
    } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
      servoPosition = leftHoodServo.getPosition() - FtcServo.SMALL_INCREMENT;
    }

    servoPosition = Range.clip(servoPosition, FtcServo.MIN_POSITION, FtcServo.MAX_POSITION);
    leftHoodServo.setPosition(servoPosition);
    rightHoodServo.setPosition(servoPosition);

    telemetry.addData("Servo", "%5.4f", servoPosition);
    telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds",
        loopTime.milliseconds(), runtime.seconds());
    telemetry.update();
    FtcUtils.sleep(10);
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
