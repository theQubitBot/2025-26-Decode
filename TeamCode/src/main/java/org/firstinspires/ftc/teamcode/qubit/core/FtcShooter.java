package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the spinning wheel.
 */
public class FtcShooter extends FtcSubSystemBase {
    private static final String TAG = "FtcShooter";
    public static final String SHOOTER_MOTOR_NAME = "shooterMotor";
    public static final double MAX_POWER = 1.0;
    public static final double MIN_POWER = -1.0;
    public static final double ZERO_POWER = 0.0;

    private final boolean shooterEnabled = true;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry = null;
    public FtcMotor motor = null;

    /* Constructor */
    public FtcShooter() {
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
        if (shooterEnabled) {
            motor = new FtcMotor(hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME));
            motor.setDirection(DcMotorEx.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            showTelemetry();
            telemetry.addData(TAG, "initialized");
        }

        FtcLogger.exit();
    }

    /**
     * Operates the wheel using the gamePads.
     *
     * @param gamePad1 The first gamePad to use.
     * @param gamePad2 The second gamePad to use.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2) {
        if (shooterEnabled && motor != null) {
            double power = gamePad1.right_trigger;
            power = Range.clip(power, MIN_POWER, MAX_POWER);
            motor.setPower(power);
        }
    }

    /**
     * Displays wheel servo telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (shooterEnabled && telemetryEnabled) {
            telemetry.addData(TAG, String.format(Locale.US, "%5.4f", motor.getPower()));
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
     * Stops the wheel.
     */
    public void stop() {
        FtcLogger.enter();
        if (shooterEnabled) {
            motor.setPower(ZERO_POWER);
        }

        FtcLogger.exit();
    }
}
