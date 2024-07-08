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

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the intake extension.
 */
public class FtcIntakeExtension extends FtcSubSystem {
    private static final String TAG = "FtcIntakeExtension";
    public static final String INTAKE_EXTENSION_SERVO_NAME = "intakeServo";
    private static final double UP_POSITION = 0.5000;
    private static final double DOWN_POSITION = 0.6000;
    private static final long UP_DOWN_TIME_MS = 500;
    private final boolean extensionEnabled = true;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry = null;
    public FtcServo intakeExtensionServo = null;

    /**
     * Initialize standard Hardware interfaces
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        if (extensionEnabled) {
            intakeExtensionServo = new FtcServo(hardwareMap.get(Servo.class, INTAKE_EXTENSION_SERVO_NAME));
            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    /**
     * Moves the intake extension up.
     *
     * @param waitTillUp When true, waits till the intake extension is fully up.
     */
    public void moveUp(boolean waitTillUp) {
        FtcLogger.enter();
        if (extensionEnabled) {
            intakeExtensionServo.setPosition(UP_POSITION);
            if (waitTillUp) {
                FtcUtils.sleep(UP_DOWN_TIME_MS);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Moves the intake extension down.
     *
     * @param waitTillDown When true, waits till the intake extension is fully down.
     */
    public void moveDown(boolean waitTillDown) {
        FtcLogger.enter();
        if (extensionEnabled) {
            intakeExtensionServo.setPosition(DOWN_POSITION);
            if (waitTillDown) {
                FtcUtils.sleep(UP_DOWN_TIME_MS);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Operates the intake extension using the gamePads.
     * Left bumper -> move the intake extension up
     * Left trigger -> move the intake extension down
     *
     * @param gamePad1 The first gamePad to use.
     * @param gamePad2 The second gamePad to use.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2) {
        if (gamePad1.right_bumper || gamePad2.right_bumper) {
            moveUp(false);
        } else if (gamePad1.left_trigger >= 0.05 || gamePad2.left_trigger >= 0.05) {
            moveDown(false);
        }
    }

    /**
     * Emits intake extension telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (extensionEnabled && telemetryEnabled) {
            telemetry.addData(TAG, String.format(Locale.US, "Position %5.4f",
                    intakeExtensionServo.getPosition()));
        }

        FtcLogger.exit();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        if (extensionEnabled) {
            intakeExtensionServo.getController().pwmEnable();
            moveUp(false);
        }

        FtcLogger.exit();
    }

    /**
     * Stops the intake extension.
     */
    public void stop() {
        FtcLogger.enter();
        if (extensionEnabled) {
            intakeExtensionServo.getController().pwmDisable();
        }

        // There is no need to explicitly move the servo upon stop as
        // the power is cut off to servo when Auto/Tele Op ends.
        FtcLogger.exit();
    }
}
