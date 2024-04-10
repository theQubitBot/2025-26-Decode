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

package org.firstinspires.ftc.teamcode.centerStage.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerStage.core.enumerations.DriveTrainEnum;
import org.firstinspires.ftc.teamcode.centerStage.core.enumerations.DriveTypeEnum;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * A class to manage the robot. This is essentially a composite design pattern.
 * Robot level operations are simply invoked on all subsystems.
 */
public class FtcBot extends FtcSubSystem {
    private static final String TAG = "FtcBot";
    private boolean telemetryEnabled = true;
    public FtcBulkRead bulkRead = null;
    public MatchConfig config = null;
    public FtcBlinkinLed blinkinLed = null;
    public FtcDriveTrain driveTrain = null;

    // robot sub systems
    public FtcHand hand = null;
    public FtcArm arm = null;
    public FtcImu imu = null;
    public FtcCatapult catapult = null;
    public FtcHook hook = null;
    public FtcLift lift = null;
    public FtcOpenCvCam openCvCam = null;
    private Telemetry telemetry = null;

    // PERFORMANCE
    // Set to true to enable parallel robot operations.
    private final boolean parallelExecutionEnabled = false;
    private ExecutorService executorService = null;
    private List<Callable<String>> callableTasks = null;
    private List<Future<String>> futures = null;

    /* Constructor */
    public FtcBot() {
        if (parallelExecutionEnabled) {
            executorService = Executors.newFixedThreadPool(8);
            callableTasks = new ArrayList<>();
        }
    }

    public void disableTelemetry() {
        FtcLogger.enter();
        telemetryEnabled = false;
        arm.telemetryEnabled = false;
        blinkinLed.telemetryEnabled = false;
        catapult.telemetryEnabled = false;
        driveTrain.telemetryEnabled = false;
        hand.telemetryEnabled = false;
        hook.telemetryEnabled = false;
        imu.telemetryEnabled = false;
        lift.telemetryEnabled = false;
        FtcLogger.exit();
    }

    public void enableTelemetry() {
        FtcLogger.enter();
        telemetryEnabled = true;
        arm.telemetryEnabled = true;
        blinkinLed.telemetryEnabled = true;
        catapult.telemetryEnabled = true;
        driveTrain.telemetryEnabled = true;
        hand.telemetryEnabled = true;
        hook.telemetryEnabled = true;
        imu.telemetryEnabled = true;
        lift.telemetryEnabled = true;
        FtcLogger.exit();
    }

    /**
     * Initialize standard Hardware interfaces
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     * @param autoOp      If true, initializes the webcams. Otherwise initializes only IMU.
     *                    Typically you would save on processing by not initializing the webcams or IMU
     *                    unnecessarily
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
        FtcLogger.enter();
        // Save reference to Hardware map
        this.telemetry = telemetry;

        bulkRead = new FtcBulkRead();
        bulkRead.init(hardwareMap, telemetry);
        config = new MatchConfig();
        config.init(hardwareMap, telemetry);
        driveTrain = new FtcDriveTrain(this);
        driveTrain.setDriveTypeAndMode(DriveTrainEnum.MECANUM_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
        driveTrain.init(hardwareMap, telemetry);
        arm = new FtcArm();
        arm.init(hardwareMap, telemetry, this);
        blinkinLed = new FtcBlinkinLed();
        blinkinLed.init(hardwareMap, telemetry);
        catapult = new FtcCatapult();
        catapult.init(hardwareMap, telemetry, this);
        hand = new FtcHand();
        hand.init(hardwareMap, telemetry, this);
        hook = new FtcHook();
        hook.init(hardwareMap, telemetry);
        imu = new FtcImu();
        if (!autoOp) {
            // PERFORMANCE
            imu.init(hardwareMap, telemetry);
        }

        lift = new FtcLift();
        lift.init(hardwareMap, telemetry, this);

        openCvCam = new FtcOpenCvCam();
        if (autoOp) {
            // PERFORMANCE
            // Don't initialize webCam in TeleOp, to save on CPU cycles and battery.
            openCvCam.init(hardwareMap, telemetry);
        }

        telemetry.addData(TAG, "initialized");
        FtcLogger.exit();
    }

    /**
     * Operate the robot in tele operation.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
        FtcLogger.enter();

        bulkRead.clearBulkCache();

        if (parallelExecutionEnabled) {
            callableTasks.clear();

            Callable<String> armTask = () -> {
                arm.operate(gamePad1, gamePad2);
                return "armTask";
            };


            Callable<String> catapultTask = () -> {
                catapult.operate(gamePad1, gamePad2, runtime);
                return "catapultTask";
            };

            Callable<String> driveTrainTask = () -> {
                driveTrain.operate(gamePad1, gamePad2, loopTime);
                return "driveTrainTask";
            };

            Callable<String> handTask = () -> {
                hand.operate(gamePad1, gamePad2);
                return "handTask";
            };

            Callable<String> hookTask = () -> {
                hook.operate(gamePad1, gamePad2, runtime);
                return "hookTask";
            };


            Callable<String> liftTask = () -> {
                lift.operate(gamePad1, gamePad2);
                return "liftTask";
            };

            callableTasks.add(armTask);
            callableTasks.add(catapultTask);
            callableTasks.add(driveTrainTask);
            callableTasks.add(handTask);
            callableTasks.add(hookTask);
            callableTasks.add(liftTask);

            try {
                futures = executorService.invokeAll(callableTasks);
            } catch (InterruptedException ignored) {
            }
        } else {
            // Arm operation
            arm.operate(gamePad1, gamePad2);

            // catapult operation
            catapult.operate(gamePad1, gamePad2, runtime);

            // Drive operation
            driveTrain.operate(gamePad1, gamePad2, loopTime);

            // Hand operation
            hand.operate(gamePad1, gamePad2);

            // hook operation
            hook.operate(gamePad1, gamePad2, runtime);

            // Lift operation
            lift.operate(gamePad1, gamePad2);
        }

        if (telemetryEnabled) {
            imu.showTelemetry();
            showGamePadTelemetry(gamePad1);
            driveTrain.showTelemetry();
            arm.showTelemetry();
            catapult.showTelemetry();
            hand.showTelemetry();
            hook.showTelemetry();
            lift.showTelemetry();
        }

        FtcLogger.exit();
    }

    /**
     * Display game pad telemetry.
     *
     * @param gamePad The gamePad.
     */
    public void showGamePadTelemetry(Gamepad gamePad) {
        FtcLogger.enter();
        if (telemetryEnabled) {
            telemetry.addData("LeftStick", "%.2f %.2f",
                    gamePad.left_stick_x, gamePad.left_stick_y);
            telemetry.addData("RightStick", "%.2f, %.2f",
                    gamePad.right_stick_x, gamePad.right_stick_y);
        }

        FtcLogger.exit();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        arm.start();
        catapult.start();
        hand.start();
        hook.start();
        FtcLogger.exit();
    }

    public void stop() {
        FtcLogger.enter();
        if (parallelExecutionEnabled) {
            executorService.shutdown();
        }

        if (imu != null) {
            imu.stop();
        }

        // Stop the async task based sub systems first to stop all async task threads.
        if (arm != null) {
            arm.stop();
        }

        if (catapult != null) {
            catapult.stop();
        }

        if (hand != null) {
            hand.stop();
        }

        if (driveTrain != null) {
            driveTrain.stop();
        }

        if (hook != null) {
            hook.stop();
        }

        if (lift != null) {
            lift.stop();
        }

        FtcLogger.exit();
    }
}
