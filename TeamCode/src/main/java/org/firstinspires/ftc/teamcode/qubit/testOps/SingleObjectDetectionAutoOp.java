package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcOpenCvCam;
import org.firstinspires.ftc.teamcode.qubit.core.FtcWebcam;
import org.firstinspires.ftc.teamcode.qubit.core.SingleObjectDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;

@Disabled
@Autonomous(group = "OpenCV")
public class SingleObjectDetectionAutoOp extends LinearOpMode {
    private OpenCvWebcam openCvWebcam;

    private double CrLowerUpdate;
    private double CbLowerUpdate;
    private double CrUpperUpdate;
    private double CbUpperUpdate;

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    @Override
    public void runOpMode() {
        telemetry.addData(">", "Initializing. Please wait...");
        telemetry.update();

        CrLowerUpdate = SingleObjectDetectionPipeline.scalarLowerYCrCb.val[1];
        CbLowerUpdate = SingleObjectDetectionPipeline.scalarLowerYCrCb.val[2];
        CrUpperUpdate = SingleObjectDetectionPipeline.scalarUpperYCrCb.val[1];
        CbUpperUpdate = SingleObjectDetectionPipeline.scalarUpperYCrCb.val[2];

        // OpenCV webcam
        openCvWebcam = FtcOpenCvCam.createWebcam(hardwareMap, FtcOpenCvCam.WEBCAM_1_NAME);

        //OpenCV Pipeline
        SingleObjectDetectionPipeline openCvPipeline;
        openCvWebcam.setPipeline(openCvPipeline = new SingleObjectDetectionPipeline(
                FtcWebcam.borderLeftX, FtcWebcam.borderRightX, FtcWebcam.borderTopY, FtcWebcam.borderBottomY));

        // Configuration of Pipeline
        openCvPipeline.configureScalarLower(
                SingleObjectDetectionPipeline.scalarLowerYCrCb.val[0],
                SingleObjectDetectionPipeline.scalarLowerYCrCb.val[1],
                SingleObjectDetectionPipeline.scalarLowerYCrCb.val[2]);
        openCvPipeline.configureScalarUpper(
                SingleObjectDetectionPipeline.scalarUpperYCrCb.val[0],
                SingleObjectDetectionPipeline.scalarUpperYCrCb.val[1],
                SingleObjectDetectionPipeline.scalarUpperYCrCb.val[2]);

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
            openCvPipeline.configureBorders(
                    FtcWebcam.borderLeftX, FtcWebcam.borderRightX, FtcWebcam.borderTopY, FtcWebcam.borderBottomY);
            if (openCvPipeline.error) {
                telemetry.addData("Exception: ", openCvPipeline.debug);
            } else {
                telemetry.addData("Frame Count", openCvWebcam.getFrameCount());
                telemetry.addData("FPS", String.format(Locale.US, "%.2f", openCvWebcam.getFps()));
                telemetry.addData("Total frame time ms", openCvWebcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", openCvWebcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", openCvWebcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", openCvWebcam.getCurrentPipelineMaxFps());
            }

            // Only use this line of the code when you want to find the lower and upper values
            testing(openCvPipeline);

            telemetry.addData("RectArea: ", openCvPipeline.getRectArea());
            telemetry.update();

            if (openCvPipeline.getRectArea() > 2000) {
                if (openCvPipeline.getRectMidpointX() > 400) {
                    Barcode_C();
                } else if (openCvPipeline.getRectMidpointX() > 200) {
                    Barcode_B();
                } else {
                    Barcode_A();
                }
            }
        }
    }

    public void testing(SingleObjectDetectionPipeline myPipeline) {
        if (lowerRuntime + 0.05 < getRuntime()) {
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerRuntime = getRuntime();
        }
        if (upperRuntime + 0.05 < getRuntime()) {
            CrUpperUpdate += -gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperRuntime = getRuntime();
        }

        CrLowerUpdate = Range.clip(CrLowerUpdate, 16, 240);
        CrUpperUpdate = Range.clip(CrUpperUpdate, 16, 240);
        CbLowerUpdate = Range.clip(CbLowerUpdate, 16, 240);
        CbUpperUpdate = Range.clip(CbUpperUpdate, 16, 240);

        myPipeline.configureScalarLower(16.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.configureScalarUpper(240.0, CrUpperUpdate, CbUpperUpdate);

        telemetry.addData("lowerCr ", (int) CrLowerUpdate);
        telemetry.addData("lowerCb ", (int) CbLowerUpdate);
        telemetry.addData("UpperCr ", (int) CrUpperUpdate);
        telemetry.addData("UpperCb ", (int) CbUpperUpdate);
    }


    public void Barcode_A() {
        telemetry.addLine("Barcode A");
    }

    public void Barcode_B() {
        telemetry.addLine("Barcode B");
    }

    public void Barcode_C() {
        telemetry.addLine("Barcode C");
    }
}
