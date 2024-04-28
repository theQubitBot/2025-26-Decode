/* Copyright (c) 2023 Viktor Taylor. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
public class FtcShooter extends FtcSubSystem {
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
            telemetry.addData(TAG, String.format(Locale.US, "Power %3.2f", motor.getPower()));
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
