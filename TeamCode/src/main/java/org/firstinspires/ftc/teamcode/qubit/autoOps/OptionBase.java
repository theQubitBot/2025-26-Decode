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

package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcDriveTrain;
import org.firstinspires.ftc.teamcode.qubit.core.FtcImu;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLift;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcMotor;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTrainEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTypeEnum;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

/**
 * A base class to provide common variables and methods.
 */
public class OptionBase {
    protected static final double RADIAN0;
    protected static final double RADIAN15;
    protected static final double RADIAN20;
    protected static final double RADIAN30;
    protected static final double RADIAN45;
    protected static final double RADIAN60;
    protected static final double RADIAN75;
    protected static final double RADIAN90;
    protected static final double RADIAN105;
    protected static final double RADIAN120;
    protected static final double RADIAN135;
    protected static final double RADIAN150;
    protected static final double RADIAN180;
    protected final long CRAWL_TIME_TO_BUCKET = 100; // milliseconds
    private final long CRAWL_TIME_TO_SUBMERSIBLE = 500; // milliseconds
    protected LinearOpMode autoOpMode;
    protected FtcBot robot;
    protected MecanumDrive drive;

    public Pose2d startPose, pose1, pose2, pose3, pose4, pose5, pose6,
            pose7, pose8, pose9, pose10, pose11, pose12;
    protected Vector2d v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12;
    protected TrajectoryActionBuilder tab1, tab2, tab3, tab4, tab5, tab6,
            tab7, tab8, tab9, tab10, tab11, tab12;
    protected Action a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12;

    static {
        RADIAN0 = Math.toRadians(0);
        RADIAN15 = Math.toRadians(15);
        RADIAN20 = Math.toRadians(20);
        RADIAN30 = Math.toRadians(30);
        RADIAN45 = Math.toRadians(45);
        RADIAN60 = Math.toRadians(60);
        RADIAN75 = Math.toRadians(75);
        RADIAN90 = Math.toRadians(90);
        RADIAN105 = Math.toRadians(105);
        RADIAN120 = Math.toRadians(120);
        RADIAN135 = Math.toRadians(135);
        RADIAN150 = Math.toRadians(150);
        RADIAN180 = Math.toRadians(180);
    }

    public OptionBase(LinearOpMode autoOpMode, FtcBot robot, MecanumDrive drive) {
        this.autoOpMode = autoOpMode;
        this.robot = robot;
        this.drive = drive;
    }

    public boolean saveAndTest() {
        // Save settings for use by TeleOp
        FtcLift.endAutoOpLeftLiftPosition = robot.lift.getLeftPosition();
        FtcLift.endAutoOpRightLiftPosition = robot.lift.getRightPosition();
        if (robot.driveTrain.driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE &&
                robot.driveTrain.driveTypeEnum == DriveTypeEnum.FIELD_ORIENTED_DRIVE) {
            robot.imu.read();
            FtcImu.endAutoOpHeading = robot.imu.getHeading();
        }

        return autoOpMode.opModeIsActive();
    }

    /**
     * Initializes variables and state.
     */
    protected void initialize() {
        FtcLogger.enter();

        FtcLogger.exit();
    }

    protected void reverseCrawlToBucket(boolean waitTillCompletion) {
        robot.driveTrain.setDrivePower(-FtcDriveTrain.FORWARD_SLO_MO_POWER, -FtcDriveTrain.FORWARD_SLO_MO_POWER,
                -FtcDriveTrain.FORWARD_SLO_MO_POWER, -FtcDriveTrain.FORWARD_SLO_MO_POWER);
        if (waitTillCompletion) {
            FtcUtils.sleep(CRAWL_TIME_TO_BUCKET);
            stopCrawling();
        }
    }

    protected void stopCrawling() {
        robot.driveTrain.setDrivePower(FtcMotor.ZERO_POWER, FtcMotor.ZERO_POWER,
                FtcMotor.ZERO_POWER, FtcMotor.ZERO_POWER);
    }
}
