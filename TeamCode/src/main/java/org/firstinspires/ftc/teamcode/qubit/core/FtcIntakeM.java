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
 * A class to manage the robot intake.
 */
public class FtcIntakeM extends FtcSubSystem {
    private static final String TAG = "FtcIntakeM";
    public static final String INTAKE_MOTOR_NAME = "intakeMotor";
    private static final double IN_POWER = 0.50;
    private static final double OUT_POWER = -0.50;
    private final boolean intakeEnabled = true;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry = null;
    public FtcMotor intakeMotor = null;

    /**
     * Initialize standard Hardware interfaces.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        if (intakeEnabled) {
            intakeMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME));
            intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
            intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    /**
     * Operates the intake using the gamePads.
     * Left bumper -> rotate out
     * Left trigger -> rotate in
     *
     * @param gamePad1 The first gamePad to use.
     * @param gamePad2 The second gamePad to use.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2) {
        FtcLogger.enter();
        if (gamePad1.left_bumper || gamePad2.left_bumper) {
            rotateOut();
        } else if (gamePad1.left_trigger >= 0.05 || gamePad2.left_trigger >= 0.05) {
            rotateIn(gamePad1, gamePad2);
        } else {
            stop();
        }

        FtcLogger.exit();
    }

    /**
     * Rotate inwards to intake the object.
     */
    private void rotateIn(Gamepad gamePad1, Gamepad gamePad2) {
        FtcLogger.enter();

        if (intakeEnabled) {
            double power = FtcMotor.ZERO_POWER;
            if (gamePad1.left_trigger >= 0.05) {
                power = gamePad1.left_trigger;
            } else if (gamePad2.left_trigger >= 0.05) {
                power = gamePad2.left_trigger;
            }

            power = Range.clip(power, FtcMotor.ZERO_POWER, FtcMotor.MAX_POWER);
            intakeMotor.setPower(power);
        }

        FtcLogger.exit();
    }

    /**
     * Rotate outwards to outtake the object.
     */
    private void rotateOut() {
        FtcLogger.enter();
        if (intakeEnabled) {
            intakeMotor.setPower(OUT_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Displays intake telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (intakeEnabled && telemetryEnabled && intakeMotor != null) {
            telemetry.addData(TAG, String.format(Locale.US, "Power: %.2f",
                    intakeMotor.getPower()));
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
     * Stops the intake.
     */
    public void stop() {
        FtcLogger.enter();
        if (intakeEnabled) {
            intakeMotor.setPower(FtcMotor.ZERO_POWER);
        }

        FtcLogger.exit();
    }
}
