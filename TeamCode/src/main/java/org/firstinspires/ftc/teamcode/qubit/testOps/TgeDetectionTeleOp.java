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
import org.firstinspires.ftc.teamcode.qubit.core.GameElement;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.GeometricShapeEnum;
import org.opencv.core.Point;

@Disabled
@TeleOp(group = "TestOp")
public class TgeDetectionTeleOp extends OpMode {
  private FtcOpenCvCam openCvCam = null;
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;

  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(">", "Initializing, please wait...");
    telemetry.update();

    openCvCam = new FtcOpenCvCam();
    openCvCam.init(hardwareMap, telemetry);
    openCvCam.modPipeline.enableAnnotations();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    dashboard.startCameraStream(openCvCam.openCvWebcam, 0);

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

    if (openCvCam.modPipeline.error) {
      telemetry.addData("Exception: ", openCvCam.modPipeline.lastException);
    } else {
      for (GameElement ge : openCvCam.modPipeline.gameElements) {
        String data = String.format("%d ", ge.attendanceCount());
        Point midPoint = FtcUtils.getMidpoint(ge.boundingRect);
        synchronized (ge) {
          if (ge.elementFound() && ge.elementConsistentlyPresent()) {
            data += String.format("Mid (%.0f, %.0f) A %.0f AR %.2f",
                midPoint.x, midPoint.y,
                ge.area, ge.aspectRatio);
            if (ge.geometricShapeEnum != GeometricShapeEnum.UNKNOWN) {
              data += String.format(" %s", ge.geometricShapeEnum.name);
            }
          }
        }

        telemetry.addData(ge.tag, data);
      }
    }

    // Show the elapsed game time.
    telemetry.addData(">", "Loop %.0f ms, cumulative %.0f seconds",
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
    if (openCvCam != null) {
      openCvCam.stop();
    }

    telemetry.addData(">", "Tele Op stopped.");
    telemetry.update();
    FtcLogger.exit();
  }
}
