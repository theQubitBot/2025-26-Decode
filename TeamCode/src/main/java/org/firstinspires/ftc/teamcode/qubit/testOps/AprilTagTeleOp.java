package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcAprilTag;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

//@Disabled
@TeleOp(group = "TestOp")
public class AprilTagTeleOp extends OpMode {
  private FtcAprilTag aprilTag;
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;

  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(">", "Initializing, please wait...");
    telemetry.update();
    aprilTag = new FtcAprilTag(null);
    aprilTag.init(hardwareMap, telemetry);

    FtcDashboard dashboard = FtcDashboard.getInstance();
    telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

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
    loopTime.reset();

    telemetry.addLine("Find lowest Exposure and highest Gain that gives reliable detection.");
    telemetry.addLine("DPad up/down:    Exposure +/-");
    telemetry.addLine("DPad left/right: Gain +/-");
    telemetry.addLine();

    int newExposure = aprilTag.currentExposure;
    int newGain = aprilTag.currentGain;
    if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
      newExposure++;
    } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
      newExposure--;
    } else if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
      newGain++;
    } else if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
      newGain--;
    }

    newExposure = Range.clip(newExposure, aprilTag.minExposure, aprilTag.maxExposure);
    newGain = Range.clip(newGain, aprilTag.minGain, aprilTag.maxGain);

    if (newExposure != aprilTag.currentExposure || newGain != aprilTag.currentGain) {
      aprilTag.updateCameraSettings(newExposure, newGain);
    }

    telemetry.addData("Exposure", "%d  (%d - %d)",
        aprilTag.currentExposure, aprilTag.minExposure, aprilTag.maxExposure);
    telemetry.addData("Gain", "%d  (%d - %d)",
        aprilTag.currentGain, aprilTag.minGain, aprilTag.maxGain);
    aprilTag.showTelemetry();

    // Show the elapsed game time.
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

    telemetry.addData(">", "Tele Op stopped.");
    telemetry.update();
    FtcLogger.exit();
  }
}
