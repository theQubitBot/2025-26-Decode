package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

  public static FollowerConstants followerConstants = new FollowerConstants()
      .mass(10.03)
      .forwardZeroPowerAcceleration(-40.0)
      .lateralZeroPowerAcceleration(-65.0)
      .useSecondaryTranslationalPIDF(false)
      .useSecondaryHeadingPIDF(false)
      .useSecondaryDrivePIDF(false)
      .centripetalScaling(0.0005)
      .holdPointTranslationalScaling(0.99)
      .holdPointHeadingScaling(0.75)
      .turnHeadingErrorThreshold(0.017453)
      .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0))
      .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0))
      .drivePIDFCoefficients(
          new FilteredPIDFCoefficients(0.005, 0, 0.00001, 0.6, 0)
      );

  public static MecanumConstants driveConstants = new MecanumConstants()
      .leftFrontMotorName("leftFrontMotor")
      .leftRearMotorName("leftRearMotor")
      .rightFrontMotorName("rightFrontMotor")
      .rightRearMotorName("rightRearMotor")
      .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
      .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
      .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
      .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
      .xVelocity(70)
      .yVelocity(52.5);

  public static PinpointConstants localizerConstants = new PinpointConstants()
      .forwardPodY(-0.83)
      .strafePodX(-0.32)
      .distanceUnit(DistanceUnit.INCH)
      .hardwareMapName("goboDriver")
      .yawScalar(1.0)
      .encoderResolution(
          GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
      )
      .customEncoderResolution(13.26291192)
      .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
      .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

  public static PathConstraints pathConstraints = new PathConstraints(
      0.995,
      1000,
      1,
      1
  );

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
        .mecanumDrivetrain(driveConstants)
        .pinpointLocalizer(localizerConstants)
        .pathConstraints(pathConstraints)
        .build();
  }
}
