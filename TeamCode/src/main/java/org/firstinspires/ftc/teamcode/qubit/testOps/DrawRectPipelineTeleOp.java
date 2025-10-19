package org.firstinspires.ftc.teamcode.qubit.testOps;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qubit.core.DrawRectPipeline;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcOpenCvCam;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@TeleOp(group = "TestOp")
public class DrawRectPipelineTeleOp extends OpMode {
  OpenCvWebcam webcam;
  DrawRectPipeline drawRectPipeline;
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;

  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(">", "Initializing, please wait...");
    telemetry.update();

    // OpenCV webcam
    webcam = FtcOpenCvCam.createWebcam(hardwareMap, FtcOpenCvCam.WEBCAM_1_NAME);

    //OpenCV Pipeline
    drawRectPipeline = new DrawRectPipeline();
    webcam.setPipeline(drawRectPipeline);

    // Timeout for obtaining permission is configurable. Set before opening.
    webcam.setMillisecondsPermissionTimeout(2500);
    webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        webcam.startStreaming(FtcOpenCvCam.CAMERA_WIDTH, FtcOpenCvCam.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
      }

      @Override
      public void onError(int errorCode) {
        /*
         * This will be called if the webcam could not be opened
         */
      }
    });

    FtcDashboard dashboard = FtcDashboard.getInstance();
    telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    dashboard.startCameraStream(webcam, 0);

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
  @SuppressLint("DefaultLocale")
  @Override
  public void loop() {
    FtcLogger.enter();
    loopTime.reset();
    drawRectPipeline.adjustRect(gamepad1);

    telemetry.addData(">", "Use left and right joysticks to reposition rectangle.");

    // Show the elapsed game time.
    telemetry.addData(">", "Loop %.0f ms, cumulative %.0f seconds",
        loopTime.milliseconds(), runtime.seconds());
    telemetry.update();
    FtcUtils.sleep(100);
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
