package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcAprilTag;
import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcMotor;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

import java.util.Locale;

// big triangle - 0.45 is max
// small triangle - 0.65 is max

@Disabled
@TeleOp(group = "TestOp")
public class CannonPowerTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;
  double newMotorPower = FtcMotor.ZERO_POWER, oldMotorPower = FtcMotor.ZERO_POWER;
  final double largeDelta = 0.10;
  final double smallDelta = 0.01;

  FtcAprilTag aprilTag;
  FtcCannon cannon;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();

    aprilTag = new FtcAprilTag(null);
    aprilTag.telemetryEnabled = true;
    aprilTag.init(hardwareMap, telemetry);

    cannon = new FtcCannon(null);
    cannon.telemetryEnabled = true;
    cannon.init(hardwareMap, telemetry);

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
    aprilTag.start();
    cannon.start();
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

    telemetry.addData("dPad up/down", "Large power +/-");
    telemetry.addData("dPad left/right", "Small power +/-");
    if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
      newMotorPower = cannon.leftCannonMotor.getPower() + largeDelta;
    } else if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
      newMotorPower = cannon.leftCannonMotor.getPower() + smallDelta;
    } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
      newMotorPower = cannon.leftCannonMotor.getPower() - largeDelta;
    } else if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
      newMotorPower = cannon.leftCannonMotor.getPower() - smallDelta;
    }

    newMotorPower = Range.clip(newMotorPower, FtcMotor.ZERO_POWER, FtcMotor.MAX_POWER);
    cannon.leftCannonMotor.setPower(newMotorPower);
    cannon.rightCannonMotor.setPower(newMotorPower);
    oldMotorPower = newMotorPower;

    if (gamepad1.xWasPressed()) {
      cannon.leftTriggerServo.setPosition(FtcCannon.LEFT_TRIGGER_UP_POSITION);
    } else if (gamepad1.xWasReleased()) {
      cannon.leftTriggerServo.setPosition(FtcCannon.LEFT_TRIGGER_DOWN_POSITION);
    } else if (gamepad1.bWasPressed()) {
      cannon.rightTriggerServo.setPosition(FtcCannon.RIGHT_TRIGGER_UP_POSITION);
    } else if (gamepad1.bWasReleased()) {
      cannon.rightTriggerServo.setPosition(FtcCannon.RIGHT_TRIGGER_DOWN_POSITION);
    }

    telemetry.addData(FtcUtils.TAG, String.format(Locale.US, "Power %.2f", newMotorPower));
    telemetry.addData(FtcUtils.TAG, String.format(Locale.US, "Velocity %5.2f, %5.2f",
        cannon.leftCannonMotor.getVelocity(), cannon.rightCannonMotor.getVelocity()));
    aprilTag.showTelemetry();

    telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds",
        loopTime.milliseconds(), runtime.seconds());
    telemetry.update();
    FtcLogger.exit();
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    cannon.stop();
    this.telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
    this.telemetry.update();
    FtcLogger.exit();
  }
}
