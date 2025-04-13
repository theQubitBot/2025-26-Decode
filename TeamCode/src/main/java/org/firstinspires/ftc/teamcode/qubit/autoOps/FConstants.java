package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        Follower.drawOnDashboard = false;
        Follower.useTranslational = true;
        Follower.useCentripetal = true;
        Follower.useHeading = true;
        Follower.useDrive = true;

        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "leftFrontMotor";
        FollowerConstants.leftRearMotorName = "leftRearMotor";
        FollowerConstants.rightFrontMotorName = "rightFrontMotor";
        FollowerConstants.rightRearMotorName = "rightRearMotor";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 10.03;

        FollowerConstants.xMovement = 70;
        FollowerConstants.yMovement = 52.5;

        FollowerConstants.forwardZeroPowerAcceleration = -40.0;
        FollowerConstants.lateralZeroPowerAcceleration = -65.0;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1,0,0,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.005,0,0.00001,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.25;
        FollowerConstants.pathEndHeadingConstraint = 0.5;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndTimeoutConstraint = 1000;

        FollowerConstants.holdPointTranslationalScaling = 0.99;
        FollowerConstants.holdPointHeadingScaling = 0.75;

        FollowerConstants.turnHeadingErrorThreshold = 0.017453; // radians == 1 degree
    }
}
