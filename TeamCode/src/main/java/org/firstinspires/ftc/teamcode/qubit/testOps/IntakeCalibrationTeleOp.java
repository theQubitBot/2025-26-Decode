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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcIntake;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcServo;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

//@Disabled
@TeleOp(group = "TestOp")
public class IntakeCalibrationTeleOp extends OpMode {
    // Declare OpMode members
    private ElapsedTime runtime = null;
    private ElapsedTime loopTime = null;

    FtcServo leftSpinServo = null;
    FtcServo rightSpinServo = null;
    FtcServo verticalSpinServo = null;
    FtcServo leftFlipServo = null;
    FtcServo rightFlipServo = null;
    double leftFlipPosition, rightFlipPosition;

    double leftSpinPower, rightSpinPower, verticalSpinPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        FtcLogger.enter();
        telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
        telemetry.update();

        leftSpinServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.LEFT_SPIN_SERVO_NAME));
        if (leftSpinServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
            leftSpinServo.getController().pwmEnable();
        }

        leftSpinServo.setDirection(Servo.Direction.REVERSE);
        leftSpinPower = FtcIntake.SPIN_STOP_POWER;
        leftSpinServo.setPosition(leftSpinPower);

        rightSpinServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.RIGHT_SPIN_SERVO_NAME));
        if (rightSpinServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
            rightSpinServo.getController().pwmEnable();
        }

        rightSpinServo.setDirection(Servo.Direction.FORWARD);
        rightSpinPower = FtcIntake.SPIN_STOP_POWER;
        rightSpinServo.setPosition(rightSpinPower);

        verticalSpinServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.VERTICAL_SPIN_SERVO_NAME));
        if (verticalSpinServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
            verticalSpinServo.getController().pwmEnable();
        }

        verticalSpinServo.setDirection(Servo.Direction.FORWARD);
        verticalSpinPower = FtcIntake.SPIN_STOP_POWER;
        verticalSpinServo.setPosition(verticalSpinPower);

        leftFlipServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.LEFT_FLIP_SERVO_NAME));
        if (leftFlipServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
            leftFlipServo.getController().pwmEnable();
        }

        leftFlipServo.setDirection(Servo.Direction.FORWARD);
        leftFlipPosition = FtcServo.MID_POSITION;
        leftFlipServo.setPosition(leftFlipPosition);

        rightFlipServo = new FtcServo(hardwareMap.get(Servo.class, FtcIntake.RIGHT_FLIP_SERVO_NAME));
        if (rightFlipServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
            rightSpinServo.getController().pwmEnable();
        }

        rightFlipServo.setDirection(Servo.Direction.REVERSE);
        rightFlipPosition = FtcServo.MID_POSITION;
        rightFlipServo.setPosition(rightFlipPosition);

        FtcLogger.exit();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData(FtcUtils.TAG, "Waiting for driver to press play");
        telemetry.update();
        FtcUtils.sleep(FtcUtils.CYCLE_MS);
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
        FtcServo servo = null;
        double position = FtcServo.MID_POSITION;

        if (gamepad1.left_trigger > 0.5) {
            leftFlipPosition -= FtcServo.LARGE_INCREMENT;
            position = leftFlipPosition;
            servo = leftFlipServo;
        } else if (gamepad1.left_bumper) {
            leftFlipPosition += FtcServo.LARGE_INCREMENT;
            position = leftFlipPosition;
            servo = leftFlipServo;
        }

        if (gamepad1.right_trigger > 0.5) {
            rightFlipPosition -= FtcServo.LARGE_INCREMENT;
            position = rightFlipPosition;
            servo = rightFlipServo;
        } else if (gamepad1.right_bumper) {
            rightFlipPosition += FtcServo.LARGE_INCREMENT;
            position = rightFlipPosition;
            servo = rightFlipServo;
        }

        if (gamepad1.left_stick_y >= 0.5 || gamepad1.left_stick_y <= -0.5) {
            leftSpinPower = Math.abs(gamepad1.left_stick_y);
            position = leftSpinPower;
            servo = leftSpinServo;
        } else {
            leftSpinPower = FtcServo.MID_POSITION;
            leftSpinServo.setPosition(leftSpinPower);
        }

        if (gamepad1.right_stick_y >= 0.5 || gamepad1.right_stick_y <= -0.5) {
            rightSpinPower = Math.abs(gamepad1.right_stick_y);
            position = rightSpinPower;
            servo = rightSpinServo;
        } else {
            rightSpinPower = FtcServo.MID_POSITION;
            rightSpinServo.setPosition(rightSpinPower);
        }

        if (gamepad1.dpad_up) {
            verticalSpinPower = FtcIntake.VERTICAL_SPIN_IN_POWER;
            position = verticalSpinPower;
            servo = verticalSpinServo;
        } else {
            verticalSpinPower = FtcServo.MID_POSITION;
            verticalSpinServo.setPosition(verticalSpinPower);
        }

        if (servo != null) {
            position = Range.clip(position, Servo.MIN_POSITION, Servo.MAX_POSITION);
            servo.setPosition(position);
        }

        telemetry.addData("Right spin", "right stick Y");
        telemetry.addData("Left spin", "left stick Y");
        telemetry.addData("Vertical spin", "dPad up");
        telemetry.addData("Right flip", "right trigger/bumper");
        telemetry.addData("Left flip", "left trigger/bumper");
        telemetry.addLine();
        telemetry.addData("Position", "LSpin %5.4f RSpin %5.4f",
                leftSpinPower, rightSpinPower);
        telemetry.addData("Position", "Lflip %5.4f Rflip %5.4f",
                leftFlipPosition, rightFlipPosition);
        telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds",
                loopTime.milliseconds(), runtime.seconds());
        telemetry.update();
        FtcUtils.sleep(FtcUtils.CYCLE_MS);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        FtcLogger.enter();
        telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
        telemetry.update();
        FtcLogger.exit();
    }
}
