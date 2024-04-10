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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Assert;

/**
 * A class to manage the external Adafruit IMU (Gyroscope).
 */
public class FtcGyro extends FtcSubSystem {
    private static final String TAG = "FtcGyro";
    static final double HEADING_THRESHOLD = 0.20;
    public boolean telemetryEnabled = true;
    Telemetry telemetry = null;
    private BNO055IMU gyro = null;
    private Orientation angles = null;
    public static double endAutoOpHeading = 0;
    private double initialTeleOpHeading = 0.0;
    private double initialTeleOpPitch = 0.0;
    private double initialTeleOpRoll = 0.0;

    public static double Heading = 0.0;
    public static double Pitch = 0.0;
    public static double Roll = 0.0;

    /* Constructor */
    public FtcGyro() {
    }

    /**
     * Determine the difference between the target heading and the robot's current heading
     *
     * @param targetHeading Desired heading (relative to global reference established at last gyro Reset).
     * @return headingOffset Degrees in the range +/- 180. Centered on the robot's frame of reference
     * Positive headingOffset means the robot should spin counter clockwise (CCW) to reduce error.
     */
    public double getHeadingOffset(double targetHeading) {
        FtcLogger.enter();
        // Z increases when robot turns left!
        // Heading is never more than 180 or less than -180

        // Normalize targetHeading to be within [-180, 179.9999]
        while (targetHeading >= 180) targetHeading -= 180.0;
        while (targetHeading <= -180) targetHeading += 180.0;

        double headingOffset;
        if (targetHeading * Heading >= 0) {
            // Target and Heading are either both positive or both negative
            headingOffset = Heading - targetHeading;
        } else {
            // They are of opposite signs
            headingOffset = targetHeading + Heading;
        }

        FtcLogger.exit();
        return headingOffset;
    }

    /**
     * Returns desired steering correction force within [-1,1] range.
     * Positive correction force implies robot should steer right
     *
     * @param headingOffset Robot heading offset in degrees
     * @param pCoEff        Proportional Gain Coefficient
     * @return The steering correction after applying the proportional gain coefficient
     */
    public double getSteeringCorrection(double headingOffset, double pCoEff) {
        FtcLogger.enter();
        double steeringCorrection = headingOffset * pCoEff;
        FtcLogger.exit();
        return steeringCorrection;
    }

    /**
     * Initialize standard Hardware interfaces.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry object to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        BNO055IMU.Parameters gyroParameters;
        gyro = hardwareMap.get(BNO055IMU.class, "adafruitImu");
        Assert.assertNotNull(gyro, "init>gyro");

        // Create gyro parameters object.
        gyroParameters = new BNO055IMU.Parameters();

        // Use degrees as angle unit.
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        // Express acceleration as m/s^2.
        gyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        // Disable logging to improve performance.
        gyroParameters.loggingEnabled = FtcUtils.DEBUG;
        gyroParameters.loggingTag = TAG;

        // Gyro must be initialized
        gyro.initialize(gyroParameters);
        while (!gyro.isGyroCalibrated()) {
            // Wait till gyro is fully calibrated.
            FtcUtils.sleep(100);
        }

        read();
        if (endAutoOpHeading == 0) {
            initialTeleOpHeading = Heading;
        } else {
            initialTeleOpHeading = endAutoOpHeading;
        }

        initialTeleOpRoll = Roll;
        initialTeleOpPitch = Pitch;

        showTelemetry();
        telemetry.addData(TAG, "initialized");
        FtcLogger.exit();
    }

    /**
     * Read the gyro and store information for later use.
     */
    public void read() {
        FtcLogger.enter();
        Assert.assertNotNull(gyro, "read>gyro");
        angles = gyro.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        Heading = angles.firstAngle;
        Roll = angles.secondAngle;
        Pitch = angles.thirdAngle;
        FtcLogger.exit();
    }

    /**
     * Display gyro telemetry.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (telemetryEnabled) {
            telemetry.addData("Heading (Z)", "%.2f, offset: %.2f",
                    Heading, initialTeleOpHeading);
            telemetry.addData("Roll", Roll);
            telemetry.addData("Pitch", Pitch);
        }

        FtcLogger.exit();
    }
}

