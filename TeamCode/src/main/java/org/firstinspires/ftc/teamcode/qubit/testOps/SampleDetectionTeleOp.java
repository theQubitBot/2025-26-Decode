package org.firstinspires.ftc.teamcode.qubit.testOps;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcOpenCvCam;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.SampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.qubit.core.SampleElement;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@TeleOp(group = "TestOp")
public class SampleDetectionTeleOp extends OpMode {
  private OpenCvWebcam openCvWebcam;
  SampleDetectionPipeline sdPipeline;
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;
  private Point midPoint;

  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();

    // OpenCV openCvWebcam
    openCvWebcam = FtcOpenCvCam.createWebcam(hardwareMap, FtcOpenCvCam.WEBCAM_1_NAME);

    //OpenCV Pipeline
    sdPipeline = new SampleDetectionPipeline(openCvWebcam);
    openCvWebcam.setPipeline(sdPipeline);

    // Timeout for obtaining permission is configurable. Set before opening.
    openCvWebcam.setMillisecondsPermissionTimeout(2500);
    openCvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        openCvWebcam.startStreaming(
            FtcOpenCvCam.CAMERA_WIDTH, FtcOpenCvCam.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
      }

      @Override
      public void onError(int errorCode) {
        /*
         * This will be called if the openCvWebcam could not be opened
         */
      }
    });

    FtcDashboard dashboard = FtcDashboard.getInstance();
    telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    dashboard.startCameraStream(openCvWebcam, 0);

    FtcLogger.exit();
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {
    telemetry.addData(FtcUtils.TAG, "Waiting for driver to press play");
    telemetry.update();
    FtcUtils.sleep(50);
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
  @SuppressLint("DefaultLocale")
  @Override
  public void loop() {
    FtcLogger.enter();
    loopTime.reset();

    if (sdPipeline.error) {
      telemetry.addData("Exception: ", sdPipeline.lastException);
    } else {
      for (SampleElement sampleElement : sdPipeline.sampleElements) {
        if (sampleElement.elementFound()) {
          midPoint = sampleElement.getMidPoint();
          telemetry.addData(FtcUtils.TAG, "%s (%f, %f)",
              sampleElement.tag, midPoint.x, midPoint.y);
        }
      }
    }

    // Show the elapsed game time.
    telemetry.addLine("");
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
    if (openCvWebcam != null) {
      openCvWebcam.stopStreaming();
      openCvWebcam.closeCameraDevice();
    }

    telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
    telemetry.update();
    FtcLogger.exit();
  }
}
