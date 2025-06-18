package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.LedColorEnum;

import java.util.Locale;

/**
 * A class to manage a REV Robotics digital LED.
 */
public class FtcLED {
    private static final String TAG = "FtcLed";
    public static final String STATUS_GREEN_LED_NAME = "statusGreenLed";
    public static final String STATUS_RED_LED_NAME = "statusRedLed";
    public static final boolean LED_HIGH = true;
    public static final boolean LED_LOW = false;
    private final boolean ledEnabled = true;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry = null;
    private DigitalChannel greenLed = null;
    private DigitalChannel redLed = null;
    private LedColorEnum currentLedColor = LedColorEnum.BLACK;

    /**
     * Initialize standard Hardware interfaces.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        if (ledEnabled) {
            greenLed = hardwareMap.get(DigitalChannel.class, STATUS_GREEN_LED_NAME);
            greenLed.setMode(DigitalChannel.Mode.OUTPUT);
            redLed = hardwareMap.get(DigitalChannel.class, STATUS_RED_LED_NAME);
            redLed.setMode(DigitalChannel.Mode.OUTPUT);
            setBlack();
            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    public void setAmber() {
        if (ledEnabled) {
            greenLed.setState(LED_LOW);
            redLed.setState(LED_LOW);
            currentLedColor = LedColorEnum.AMBER;
        }
    }

    public void setBlack() {
        if (ledEnabled) {
            greenLed.setState(LED_HIGH);
            redLed.setState(LED_HIGH);
            currentLedColor = LedColorEnum.BLACK;
        }
    }

    public void setGreen() {
        if (ledEnabled) {
            greenLed.setState(LED_HIGH);
            redLed.setState(LED_LOW);
            currentLedColor = LedColorEnum.GREEN;
        }
    }

    public void setRed() {
        if (ledEnabled) {
            greenLed.setState(LED_LOW);
            redLed.setState(LED_HIGH);
            currentLedColor = LedColorEnum.RED;
        }
    }

    /**
     * Displays LED telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (ledEnabled && telemetryEnabled && greenLed != null && redLed != null) {
            telemetry.addData(TAG, String.format(Locale.US, "%s",
                    currentLedColor.toString()));
        }

        FtcLogger.exit();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        FtcLogger.exit();
    }

    /**
     * Stops the LED.
     */
    public void stop() {
        FtcLogger.enter();
        setBlack();
        FtcLogger.exit();
    }
}
