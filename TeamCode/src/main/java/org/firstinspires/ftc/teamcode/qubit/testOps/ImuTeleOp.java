package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qubit.core.FtcImu;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

@Disabled
@TeleOp(group = "TestOp")
public class ImuTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;
  FtcImu imu = null;
  double targetHeading = 0;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();
    imu = new FtcImu();
    imu.init(hardwareMap, telemetry);
    imu.telemetryEnabled = FtcUtils.DEBUG;
    FtcImu.endAutoOpHeading = 0;
    imu.showTelemetry();

    // Inform the driver that initialization is complete.
    telemetry.update();
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

    imu.read();
    imu.showTelemetry();
    if (gamepad1.y)
      targetHeading = 0;
    else if (gamepad1.b)
      targetHeading = -90;
    else if (gamepad1.x)
      targetHeading = 90;
    else if (gamepad1.a)
      targetHeading = -180;
    telemetry.addData(FtcUtils.TAG, "Target %.1f, Heading %.1f, Offset %.1f",
        targetHeading, imu.getHeading(), imu.getHeadingOffset(targetHeading));
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
    imu.stop();
    telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
    telemetry.update();
    FtcLogger.exit();
  }
}
