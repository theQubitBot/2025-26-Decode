package org.firstinspires.ftc.teamcode.qubit.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcImu;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

@Disabled
@TeleOp(group = "Official")
public class DriverTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;
  private double lastLoopTime = 0.0;
  FtcBot robot = null;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();
    robot = new FtcBot();
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
    FtcLogger.enter();
    loopTime.reset();
    robot.operate(gamepad1, gamepad2, lastLoopTime, runtime);

    // Show the elapsed game time.
    telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds",
        loopTime.milliseconds(), runtime.seconds());
    lastLoopTime = loopTime.milliseconds();
    FtcLogger.exit();
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    if (robot != null) {
      robot.stop();
    }

    // Reset endAutoOpHeading so that a manual re-execution of TeleOp during robot
    // development and testing will be unaffected by the last AutoOp execution.
    // This may be an issue if the Control Hub needs a reboot
    // in the middle of the match and FOD is enabled.
    FtcImu.endAutoOpHeading = 0;
    telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
    telemetry.update();
    FtcLogger.exit();
  }
}
