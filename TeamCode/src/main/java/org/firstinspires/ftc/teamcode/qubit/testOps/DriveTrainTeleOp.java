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

package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTrainEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTypeEnum;

@Disabled
@TeleOp(group = "TestOp")
public class DriveTrainTeleOp extends OpMode {
    // Declare OpMode members
    private ElapsedTime runtime = null;
    private ElapsedTime loopTime = null;
    private double lastLoopTime = 0.0;
    FtcBot robot = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        FtcLogger.enter();
        telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
        telemetry.update();
        robot = new FtcBot();
        robot.init(hardwareMap, telemetry, false);
        robot.driveTrain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FtcLogger.exit();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData(FtcUtils.TAG, "Waiting for driver to press play");
        telemetry.update();
        FtcUtils.sleep(10);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        FtcLogger.enter();
        telemetry.addData(FtcUtils.TAG, "Starting...");
        telemetry.update();
        runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        if (FtcUtils.DEBUG) {
            robot.enableTelemetry();
        } else {
            robot.disableTelemetry();
        }

        FtcLogger.exit();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        FtcLogger.enter();
        // Show the elapsed game time and wheel power.
        loopTime.reset();

        telemetry.addData(FtcUtils.TAG, "a: FWD POV, b: RWD POV, x: MecanumDrive FOD, y: AWD POV");
        if (gamepad1.a) {
            robot.driveTrain.setDriveTypeAndMode(
                    DriveTrainEnum.FRONT_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
        } else if (gamepad1.b) {
            robot.driveTrain.setDriveTypeAndMode(
                    DriveTrainEnum.REAR_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
        } else if (gamepad1.x) {
            robot.driveTrain.setDriveTypeAndMode(
                    DriveTrainEnum.MECANUM_WHEEL_DRIVE, DriveTypeEnum.FIELD_ORIENTED_DRIVE);
        } else if (gamepad1.y) {
            robot.driveTrain.setDriveTypeAndMode(
                    DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
        }

        robot.bulkRead.clearBulkCache();
        robot.driveTrain.operate(gamepad1, gamepad2, lastLoopTime, runtime);
        robot.driveTrain.showTelemetry();
        robot.imu.showTelemetry();
        robot.showGamePadTelemetry(gamepad1);
        telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds",
                loopTime.milliseconds(), runtime.seconds());
        telemetry.update();
        lastLoopTime = loopTime.milliseconds();
        FtcLogger.exit();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        FtcLogger.enter();
        robot.driveTrain.stop();
        telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
        telemetry.update();
        FtcLogger.exit();
    }
}
