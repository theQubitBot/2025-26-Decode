package org.firstinspires.ftc.teamcode.qubit.core;

import static org.firstinspires.ftc.teamcode.qubit.core.FtcUtils.sleep;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.TimeUnit;
import java.util.Locale;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;



/**
 * A class to manage the spinning wheel.
 */
public class FtcSorter extends FtcSubSystemBase {
    private static final String TAG = "FtcSorter";
    public static final String SORTER_MOTOR_NAME = "sorterMotor";
    public static final double MAX_POWER = 1.0;
    public static final double MIN_POWER = -1.0;
    public static final double ZERO_POWER = 0.0;

    private final boolean sorterEnabled = true;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry = null;
    public FtcMotor motor = null;

    private I2cDeviceSynch huskyLens;

    /* Constructor */
    public FtcSorter() {
    }

    /**
     * Initialize standard Hardware interfaces
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        if (sorterEnabled) {
            motor = new FtcMotor(hardwareMap.get(DcMotorEx.class, SORTER_MOTOR_NAME));
            motor.setDirection(DcMotorEx.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            showTelemetry();
            telemetry.addData(TAG, "initialized");

            // Initialize the I2C device
            huskyLens = hardwareMap.get(I2cDeviceSynch.class, "huskyLens");
            huskyLens.setI2cAddress(I2cAddr.create7bit(0x32)); // Default I2C address for HuskyLens
            huskyLens.engage();

            telemetry.addData("Status", "Initialized");
        }

        FtcLogger.exit();
    }



    /**
     * Operates the merry-go-round using the gamePads.
     *
     * @param gamePad1 The first gamePad to use.
     * @param gamePad2 The second gamePad to use.
     */

    // Possibly - have merry-go-round operate completely autonomously? using diff. kinds of sensors
    public void operate(Gamepad gamePad1, Gamepad gamePad2) {
        FtcLogger.enter();
        if (sorterEnabled && motor != null) {
            double power = gamePad1.left_trigger;
            power = Range.clip(power, MIN_POWER, MAX_POWER);
            motor.setPower(power);
        }
        FtcLogger.exit();
    }

    /**
     * Displays motor power telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (sorterEnabled && telemetryEnabled) {
            telemetry.addData(TAG, String.format(Locale.US, "%5.4f", motor.getPower()));
        }

        FtcLogger.exit();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        byte[] data = huskyLens.read(0x00, 6);
        int colorCode = data[0] & 0xFF;

        telemetry.addData("Color Code", colorCode);
        telemetry.update();

        if (colorCode == 800080) { // purple
            motor.setPower(0.5); // this is just a stand-in line of code
            // for whatever mechanical function we actually need
            // to make the artifact go to the purple cannon
        } else if (colorCode == 006400) { // green
            motor.setPower(-0.5); // this is just a stand-in line of code
            // for whatever mechanical function we actually need
            // to make the artifact go to the green cannon
        }

        sleep(100);
        FtcLogger.exit();
    }

    /**
     * Stops the merry-go-round mechanism.
     */
    public void stop() {
        FtcLogger.enter();
        if (sorterEnabled) {
            motor.setPower(ZERO_POWER);
        }

        FtcLogger.exit();
    }
}
