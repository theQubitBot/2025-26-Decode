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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * A class to manage the built-in IMU.
 */
public class FtcImu extends FtcSubSystem {
    public static final String TAG = "FtcImu";
    static final double HEADING_THRESHOLD = 0.20;
    public boolean telemetryEnabled = true;
    Telemetry telemetry = null;

    private BNO055IMU imuBno055 = null;
    private IMU imuBhi260ap = null;
    private final boolean useImuBhi260ap = false;

    // PERFORMANCE
    // Reading angular velocity takes 5ms. Enable it only if you really need it.
    private final boolean enableAngularVelocityReads = false;
    public static double endAutoOpHeading = 0;
    private double initialTeleOpHeading = 0.0;
    private double initialGyroHeadingToDetermineDrift = 0.0;
    private double initialTeleOpPitch = 0.0;
    private double initialTeleOpRoll = 0.0;

    // PERFORMANCE
    // Set to true to enable asynchronous background update of IMU values.
    public final boolean asyncUpdaterEnabled = true;
    private static final Object directionLock = new Object();
    private static final Object velocityLock = new Object();
    private FtcImuAsyncUpdater imuAsyncUpdater = null;

    private static double heading = 0.0; // Increases when robot turns left
    private static double pitch = 0.0; // Increases when front rises
    private static double roll = 0.0; // Increases when left rises

    private static double headingVelocity = 0.0;
    private static double pitchVelocity = 0.0;
    private static double rollVelocity = 0.0;

    // PERFORMANCE
    // Simple mechanism to ensure a single IMU read per loop, and only when needed.
    private boolean gyroAlreadyRead = false;

    /* Constructor */
    public FtcImu() {
    }

    /**
     * A thread safe method to get IMU heading.
     *
     * @return The IMU heading.
     */
    public double getHeading() {
        double currentHeading;
        synchronized (directionLock) {
            currentHeading = heading;
        }

        return currentHeading;
    }

    /**
     * A thread safe method to get IMU pitch.
     *
     * @return The IMU pitch.
     */
    public double getPitch() {
        double currentPitch;
        synchronized (directionLock) {
            currentPitch = pitch;
        }

        return currentPitch;
    }

    /**
     * A thread safe method to get IMU roll.
     *
     * @return The IMU roll.
     */
    public double getRoll() {
        double currentRoll;
        synchronized (directionLock) {
            currentRoll = roll;
        }

        return currentRoll;
    }

    /**
     * A thread safe method to get IMU heading velocity.
     *
     * @return The IMU heading velocity.
     */
    public double getHeadingVelocity() {
        double currentHeadingVelocity;
        synchronized (velocityLock) {
            currentHeadingVelocity = headingVelocity;
        }

        return currentHeadingVelocity;
    }

    /**
     * A thread safe method to get IMU pitch velocity.
     *
     * @return The IMU pitch velocity.
     */
    public double getPitchVelocity() {
        double currentPitchVelocity;
        synchronized (velocityLock) {
            currentPitchVelocity = pitchVelocity;
        }

        return currentPitchVelocity;
    }

    /**
     * A thread safe method to get IMU roll velocity.
     *
     * @return The IMU roll velocity.
     */
    public double getRollVelocity() {
        double currentRollVelocity;
        synchronized (velocityLock) {
            currentRollVelocity = rollVelocity;
        }

        return currentRollVelocity;
    }

    /**
     * Determine the difference between the target heading and the robot's current heading
     *
     * @param targetHeading Desired heading (relative to global reference established at last gyro Reset).
     * @return headingOffset Degrees in the range +/- 180. Centered on the robot's frame of reference
     * Positive headingOffset means the robot should spin Counter Clockwise (CCW) to reduce error.
     */
    public double getHeadingOffset(double targetHeading) {
        FtcLogger.enter();

        // Z increases when robot turns left!
        // Yaw is never more than 180 or less than -180
        targetHeading = normalizeHeading(targetHeading);
        double currentHeading = getHeading();
        double headingOffset;
        if (targetHeading * currentHeading >= 0) {
            // Target and current are either both positive or both negative
            headingOffset = currentHeading - targetHeading;
        } else {
            // They are of opposite signs
            headingOffset = targetHeading + currentHeading;
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
     * Initialize IMU. Robot must be still during IMU initialization (calibration).
     * See <a href="https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html">...</a>
     * for initialization times.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry object to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        // Save reference to Hardware map
        this.telemetry = telemetry;
        if (useImuBhi260ap) {
            imuBhi260ap = hardwareMap.get(IMU.class, "imu");
            if (imuBhi260ap != null) {
                RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
                imuBhi260ap.initialize(new IMU.Parameters(orientationOnRobot));
                imuBhi260ap.resetYaw();
            }
        } else {
            imuBno055 = hardwareMap.get(BNO055IMU.class, "adafruitImu");
            if (imuBno055 != null) {
                // Create new IMU Parameters object.
                BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

                // Use degrees as angle unit.
                imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

                // Express acceleration as m/s^2.
                imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

                // Disable logging to improve performance.
                imuParameters.loggingEnabled = FtcUtils.DEBUG;
                imuParameters.loggingTag = TAG;

                // IMU must be initialized
                imuBno055.initialize(imuParameters);
                while (!imuBno055.isGyroCalibrated()) {
                    // Wait till gyro is fully calibrated.
                    FtcUtils.sleep(100);
                }
            }
        }

        if ((useImuBhi260ap && imuBhi260ap != null) || imuBno055 != null) {
            // Get a reading before the async reader is started.
            readAsync();
            if (asyncUpdaterEnabled) {
                if (imuAsyncUpdater != null) {
                    // Stop any previously executing updater
                    imuAsyncUpdater.stop();
                }

                imuAsyncUpdater = new FtcImuAsyncUpdater(this);
                Thread imuUpdaterThread = new Thread(imuAsyncUpdater);
                imuUpdaterThread.start();
            }

            if (endAutoOpHeading == 0) {
                initialTeleOpHeading = getHeading();
            } else {
                initialTeleOpHeading = endAutoOpHeading;
            }

            initialTeleOpRoll = getRoll();
            initialTeleOpPitch = getPitch();

            initialGyroHeadingToDetermineDrift = getHeading();
            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not initialized");
        }

        FtcLogger.exit();
    }

    /**
     * A simple logic to determine if IMU is drifting.
     *
     * @return True, if IMU is drifting.
     */
    public boolean isGyroDrifting() {
        read();
        return !FtcUtils.areEqual(
                initialGyroHeadingToDetermineDrift, getHeading(), FtcUtils.EPSILON1);
    }

    /**
     * Normalize heading to be within [-180, 179.9999] degrees.
     *
     * @param heading The heading (in degrees) to normalize.
     * @return Normalized heading.
     */
    public static double normalizeHeading(double heading) {
        while (heading >= 180) heading -= 180.0;
        while (heading <= -180) heading += 180.0;
        return heading;
    }

    /**
     * Read the IMU and store information for later use.
     */
    public void read() {
        if (!asyncUpdaterEnabled) {
            readAsync();
        }
    }

    /**
     * Read the IMU and store information for later use.
     * Invoked by async updater directly.
     * This may be invoked from other classes.
     */
    public void readAsync() {
        if (useImuBhi260ap && imuBhi260ap != null) {
            YawPitchRollAngles yawPitchRollAngles = imuBhi260ap.getRobotYawPitchRollAngles();
            synchronized (directionLock) {
                heading = yawPitchRollAngles.getYaw(AngleUnit.DEGREES);
                pitch = yawPitchRollAngles.getPitch(AngleUnit.DEGREES);
                roll = yawPitchRollAngles.getRoll(AngleUnit.DEGREES);
            }

            if (enableAngularVelocityReads) {
                AngularVelocity angularVelocity = imuBhi260ap
                        .getRobotAngularVelocity(AngleUnit.DEGREES);
                synchronized (velocityLock) {
                    headingVelocity = angularVelocity.zRotationRate;
                    pitchVelocity = angularVelocity.xRotationRate;
                    rollVelocity = angularVelocity.yRotationRate;
                }
            }
        } else if (imuBno055 != null) {
            Orientation orientation = imuBno055
                    .getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            synchronized (directionLock) {
                heading = orientation.firstAngle;
                roll = orientation.secondAngle;
                pitch = orientation.thirdAngle;
            }

            if (enableAngularVelocityReads) {
                AngularVelocity angularVelocity = imuBno055.getAngularVelocity();
                synchronized (velocityLock) {
                    headingVelocity = angularVelocity.zRotationRate;
                    pitchVelocity = angularVelocity.yRotationRate;
                    rollVelocity = angularVelocity.xRotationRate;
                }
            }
        }
    }

    /**
     * Read the IMU if not already read.
     */
    public void readOnce() {
        if (!gyroAlreadyRead) {
            read();
            gyroAlreadyRead = true;
        }
    }

    /**
     * If the IMU has already been read, then reset so that another read may occur.
     */
    public void resetReadOnce() {
        gyroAlreadyRead = false;
    }

    /**
     * Display IMU (gyro) telemetry.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (telemetryEnabled &&
                ((useImuBhi260ap && imuBhi260ap != null) || imuBno055 != null)) {
            telemetry.addData("Heading/Yaw", "%.2f, offset: %.2f, hV: %.2f",
                    getHeading(), initialTeleOpHeading, getHeadingVelocity());
            telemetry.addData("Pitch", "%.2f, pV: %.2f", getPitch(), getPitchVelocity());
            telemetry.addData("Roll", "%.2f, rV: %.2f", getRoll(), getRollVelocity());
        }

        FtcLogger.exit();
    }

    /**
     * Stops the background IMU updater, if enabled.
     */
    public void stop() {
        FtcLogger.enter();
        if (asyncUpdaterEnabled && imuAsyncUpdater != null) {
            imuAsyncUpdater.stop();
            imuAsyncUpdater = null;
        }

        FtcLogger.exit();
    }
}
