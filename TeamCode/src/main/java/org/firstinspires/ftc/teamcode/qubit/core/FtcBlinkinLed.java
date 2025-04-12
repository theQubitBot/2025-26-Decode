/* Copyright (c) 2024 The Qubit Bot. All rights reserved.
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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;

import java.util.Locale;

/**
 * A class to manage the REV Blinkin LED strip.
 */
public class FtcBlinkinLed extends FtcSubSystem {
    private static final String TAG = "FtcBlinkinLed";
    private static final String BLINKIN_NAME = "blinkinLed";
    private final boolean blinkinLedEnabled = true;
    public boolean telemetryEnabled = true;
    private RevBlinkinLedDriver blinkinLedDriver = null;

    // Start with LED strip being off.
    private BlinkinPattern currentPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    private Telemetry telemetry;
    private FtcBot parent = null;

    public FtcBlinkinLed(FtcBot robot) {
        parent = robot;
    }

    /**
     * Initialize standard Hardware interfaces
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        if (blinkinLedEnabled) {
            blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, BLINKIN_NAME);
            set(currentPattern);
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }
    }

    public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
        FtcLogger.enter();
        if (blinkinLedEnabled) {
            if (!FtcUtils.DEBUG && FtcUtils.gameOver(runtime)) {
                stop();
            } else if (gamePad1.right_bumper || gamePad2.right_bumper) {
                if (parent != null) {
                    if (!parent.rnp.isRetracted() ||
                            !parent.lift.atPosition(FtcLift.POSITION_FLOOR) ||
                            !parent.arm.isForward()) {
                        parent.blinkinLed.set(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
                    } else {
                        stop();
                    }
                }
            } else if (gamePad1.left_bumper || gamePad2.left_bumper) {
                if (parent != null) {
                    if (parent.intake.isDelivering()) {
                        parent.blinkinLed.set(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
                    } else {
                        stop();
                    }
                }
            } else if (FtcUtils.lastNSeconds(runtime, 10)) {
                if (parent != null) {
                    if (parent.config.allianceColor == AllianceColorEnum.BLUE) {
                        if (currentPattern != BlinkinPattern.HEARTBEAT_BLUE) {
                            set(BlinkinPattern.HEARTBEAT_BLUE);
                        }
                    } else {
                        if (currentPattern != BlinkinPattern.HEARTBEAT_RED) {
                            set(BlinkinPattern.HEARTBEAT_RED);
                        }
                    }
                } else {
                    if (currentPattern != BlinkinPattern.HEARTBEAT_WHITE) {
                        set(BlinkinPattern.HEARTBEAT_WHITE);
                    }
                }
            } else {
                stop();
            }
        }

        FtcLogger.exit();
    }

    /**
     * Set the next pattern.
     */
    public void setNextPattern() {
        set(currentPattern.next());
    }

    /**
     * Set the previous pattern.
     */
    public void setPreviousPattern() {
        set(currentPattern.previous());
    }

    /**
     * Sets the LED display pattern.
     *
     * @param pattern The display pattern to set to.
     */
    public void set(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (blinkinLedEnabled && blinkinLedDriver != null) {
            currentPattern = pattern;
            blinkinLedDriver.setPattern(pattern);
        }
    }

    /**
     * Displays LED telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        if (blinkinLedEnabled && telemetryEnabled && blinkinLedDriver != null) {
            telemetry.addData(TAG, String.format(Locale.US, "%s (%d)",
                    currentPattern.toString(), currentPattern.ordinal()));
        }
    }

    /**
     * Stops the current LED display pattern by setting the BLACK pattern.
     */
    public void stop() {
        set(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
}
