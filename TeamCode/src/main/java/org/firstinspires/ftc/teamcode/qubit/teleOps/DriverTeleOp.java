package org.firstinspires.ftc.teamcode.qubit.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;

//@Disabled
@TeleOp(group = "Official")
public class DriverTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;
  private double lastLoopTime = 1;
  BaseBot robot = null;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();
    robot = BaseBot.getBot();
    robot.init(hardwareMap, telemetry, false);
    robot.driveTrain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    FtcLogger.exit();
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {
    telemetry.addData(FtcUtils.TAG, "Waiting for driver to press play");
    telemetry.addLine();
    robot.localizer.showTelemetry();
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

    loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    robot.start();

    if (FtcUtils.DEBUG) {
      robot.enableTelemetry();
    } else {
      robot.disableTelemetry();
    }

    FtcLogger.exit();
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {
    loopTime.reset();
    FtcLogger.enter();
    robot.operate(gamepad1, gamepad2, lastLoopTime, runtime);

    // Show the elapsed game time.
    telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds",
        lastLoopTime, runtime.seconds());
    FtcLogger.exit();
    lastLoopTime = loopTime.milliseconds();
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    robot.stop();

    telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
    telemetry.update();
    FtcLogger.exit();
  }
}
