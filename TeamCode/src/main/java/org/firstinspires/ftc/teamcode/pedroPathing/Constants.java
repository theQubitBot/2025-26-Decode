package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class Constants {
  // brakingStart of 0.2 for score path end
  // 1 for start and 1 for strength is good for intake
  public static double HARD_BRAKE_STRENGTH = 1.1; // higher value -> more braking
  public static double SOFT_BRAKE_STRENGTH = 1.0;
  public static double HARD_BRAKE_START = 0.9; // lower value -> late braking
  public static double SOFT_BRAKE_START = 1.0;

  public static FollowerConstants followerConstants = new FollowerConstants()
      .mass(12.7)
      .forwardZeroPowerAcceleration(-27.66)
      .lateralZeroPowerAcceleration(-70.0)
      .useSecondaryTranslationalPIDF(false)
      .useSecondaryHeadingPIDF(false)
      .useSecondaryDrivePIDF(false)
      .centripetalScaling(0.0005)
      .holdPointTranslationalScaling(0.99)
      .holdPointHeadingScaling(0.75)
      .turnHeadingErrorThreshold(0.01745329251994329576923690768489) // 1 degree
      .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0))
      .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0))
      .drivePIDFCoefficients(
          new FilteredPIDFCoefficients(0.005, 0, 0.00001, 0.6, 0)
      );

  public static MecanumConstants driveConstants = new MecanumConstants()
      .maxPower(1)
      .leftFrontMotorName("leftFrontMotor")
      .leftRearMotorName("leftRearMotor")
      .rightFrontMotorName("rightFrontMotor")
      .rightRearMotorName("rightRearMotor")
      .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
      .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
      .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
      .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
      .useBrakeModeInTeleOp(true)
      .xVelocity(81.0)
      .yVelocity(45.0);

  public static PinpointConstants localizerConstants = new PinpointConstants()
      .forwardPodY(-6.5)
      .strafePodX(-1.25)
      .distanceUnit(DistanceUnit.INCH)
      .hardwareMapName("pinpointDriver")
      .yawScalar(1.0)
      .encoderResolution(
          GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
      )
      .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
      .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

  public static PathConstraints pathConstraints = new PathConstraints(
      0.995,
      1000,
      SOFT_BRAKE_STRENGTH,
      SOFT_BRAKE_START
  );

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
        .pathConstraints(pathConstraints)
        .mecanumDrivetrain(driveConstants)
        .pinpointLocalizer(localizerConstants)
        .build();
  }
}
