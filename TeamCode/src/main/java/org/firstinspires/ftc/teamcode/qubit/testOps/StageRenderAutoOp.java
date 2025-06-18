package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcOpenCvCam;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.StageRenderPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;

@Disabled
@Autonomous(group = "TestOp")
public class StageRenderAutoOp extends LinearOpMode {
    OpenCvWebcam webcam;
    StageRenderPipeline openCvPipeline;

    @Override
    public void runOpMode() {
        telemetry.addData(">", "Initializing. Please wait...");
        telemetry.update();

        webcam = FtcOpenCvCam.createWebcam(hardwareMap, FtcOpenCvCam.WEBCAM_1_NAME);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        openCvPipeline = new StageRenderPipeline();

        // Timeout for obtaining permission is configurable. Set before opening.
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(openCvPipeline);
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

        // Wait for driver to hit start
        while (opModeInInit()) {
            telemetry.addData(">", "Initialization complete. Waiting for start");
            telemetry.update();
            FtcUtils.sleep(10);
        }

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Num contours found", openCvPipeline.getContourCount());
            telemetry.addData("Pipeline stage", openCvPipeline.getStageToRenderToViewport());
            telemetry.update();

            if (gamepad1.a) {
                openCvPipeline.incrementStageToRenderToViewport();
            } else if (gamepad1.b) {
                openCvPipeline.decrementStageToRenderToViewport();
            }

            sleep(250);
        }
    }
}
