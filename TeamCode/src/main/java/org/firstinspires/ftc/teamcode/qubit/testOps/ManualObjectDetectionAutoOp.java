package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcOpenCvCam;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.ManualObjectDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Autonomous(group = "TestOp")
public class ManualObjectDetectionAutoOp extends LinearOpMode {
    private OpenCvWebcam openCvWebcam;

    @Override
    public void runOpMode() {
        telemetry.addData(">", "Initializing. Please wait...");
        telemetry.update();

        // OpenCV webcam
        openCvWebcam = FtcOpenCvCam.createWebcam(hardwareMap, FtcOpenCvCam.WEBCAM_1_NAME);

        //OpenCV Pipeline
        ManualObjectDetectionPipeline openCvPipeline;
        openCvWebcam.setPipeline(openCvPipeline = new ManualObjectDetectionPipeline());

        // Webcam Streaming
        openCvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                openCvWebcam.startStreaming(
                        FtcOpenCvCam.CAMERA_WIDTH, FtcOpenCvCam.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
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
        dashboard.startCameraStream(openCvWebcam, 0);

        telemetry.addData(">", "Initialization complete. Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (openCvPipeline.error) {
                telemetry.addData("Exception: ", openCvPipeline.lastException);
            } else {
                if (gamepad1.left_stick_x > 0) {
                    openCvPipeline.IncreaseLowerHue();
                } else if (gamepad1.left_stick_x < 0) {
                    openCvPipeline.DecreaseLowerHue();
                }

                if (gamepad1.right_stick_x > 0) {
                    openCvPipeline.IncreaseUpperHue();
                } else if (gamepad1.right_stick_x < 0) {
                    openCvPipeline.DecreaseUpperHue();
                }
            }

            telemetry.addData("Usage", "Use left and right joysticks to adjust Hue constraints");
            telemetry.addData("HSV limits", "[%.0f, %.0f]",
                    openCvPipeline.gameElement.lowerColorThreshold.val[0],
                    openCvPipeline.gameElement.upperColorThreshold.val[0]);
            telemetry.update();
            FtcUtils.sleep(100);
        }
    }
}
