package org.firstinspires.ftc.teamcode.qubit.motorOps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class MotorTuningController {
  public static double MOTOR_TICKS_PER_REV = 28;
  public static double MOTOR_MAX_RPM = 1600;
  public static double MOTOR_TEST_RPM1 = 820;
  public static double MOTOR_TEST_RPM2 = 1440;
  public static double MOTOR_GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed

  public static double TESTING_MAX_RPM = 1.0 * MOTOR_MAX_RPM;
  public static double TESTING_MIN_RPM = 0.25 * MOTOR_MAX_RPM;

  // These are prefixed with "STATE1", "STATE2", etc. because Dashboard displays variables in
  // alphabetical order. Thus, we preserve the actual order of the process
  // Then we append Z just because we want it to show below the MOTOR_ and TESTING_ because
  // these settings aren't as important
  public static double ZSTATE1_RAMPING_UP_DURATION = 4;
  public static double ZSTATE2_COASTING_1_DURATION = 4;
  public static double ZSTATE3_RAMPING_DOWN_DURATION = 2;
  public static double ZSTATE4_COASTING_2_DURATION = 2;
  public static double ZSTATE5_RANDOM_1_DURATION = 2;
  public static double ZSTATE6_RANDOM_2_DURATION = 2;
  public static double ZSTATE7_RANDOM_3_DURATION = 2;
  public static double ZSTATE8_REST_DURATION = 1;

  enum State {
    RAMPING_UP,
    COASTING_1,
    RAMPING_DOWN,
    COASTING_2,
    RANDOM_1,
    RANDOM_2,
    RANDOM_3,
    REST
  }

  private ElapsedTime stateTimer = new ElapsedTime();
  private State currentState = State.REST;
  private double targetVelocity = 0;

  public void start() {
    stateTimer.reset();
    currentState = State.REST;
    targetVelocity = 0;
  }

  public double getTargetVelocity() {
    if (currentState == State.RAMPING_UP) {
      if (stateTimer.seconds() <= ZSTATE1_RAMPING_UP_DURATION) {
        double progress = stateTimer.seconds() / ZSTATE1_RAMPING_UP_DURATION;
        double target = progress * (TESTING_MAX_RPM - TESTING_MIN_RPM) + TESTING_MIN_RPM;
        targetVelocity = rpmToTicksPerSecond(target);
      } else {
        currentState = State.COASTING_1;
        stateTimer.reset();
      }
    } else if (currentState == State.COASTING_1) {
      if (stateTimer.seconds() <= ZSTATE2_COASTING_1_DURATION) {
        targetVelocity = rpmToTicksPerSecond(TESTING_MAX_RPM);
      } else {
        currentState = State.RAMPING_DOWN;
        stateTimer.reset();
      }
    } else if (currentState == State.RAMPING_DOWN) {
      if (stateTimer.seconds() <= ZSTATE3_RAMPING_DOWN_DURATION) {
        double progress = stateTimer.seconds() / ZSTATE3_RAMPING_DOWN_DURATION;
        double target = TESTING_MAX_RPM - progress * (TESTING_MAX_RPM - TESTING_MIN_RPM);
        targetVelocity = rpmToTicksPerSecond(target);
      } else {
        currentState = State.COASTING_2;
        stateTimer.reset();
      }
    } else if (currentState == State.COASTING_2) {
      if (stateTimer.seconds() <= ZSTATE4_COASTING_2_DURATION) {
        targetVelocity = rpmToTicksPerSecond(TESTING_MIN_RPM);
      } else {
        currentState = State.RANDOM_1;
        stateTimer.reset();
        targetVelocity = rpmToTicksPerSecond(MOTOR_TEST_RPM1);
      }
    } else if (currentState == State.RANDOM_1) {
      if (stateTimer.seconds() <= ZSTATE5_RANDOM_1_DURATION) {
        // velocity was set at state change
      } else {
        currentState = State.RANDOM_2;
        stateTimer.reset();
        targetVelocity = rpmToTicksPerSecond(MOTOR_TEST_RPM1);
      }
    } else if (currentState == State.RANDOM_2) {
      if (stateTimer.seconds() <= ZSTATE6_RANDOM_2_DURATION) {
        // velocity was set at state change
      } else {
        currentState = State.RANDOM_3;
        stateTimer.reset();
        targetVelocity = rpmToTicksPerSecond(MOTOR_TEST_RPM2);
      }
    } else if (currentState == State.RANDOM_3) {
      if (stateTimer.seconds() <= ZSTATE7_RANDOM_3_DURATION) {
        // velocity was set at state change
      } else {
        currentState = State.REST;
        stateTimer.reset();
      }
    } else if (currentState == State.REST) {
      if (stateTimer.seconds() <= ZSTATE8_REST_DURATION) {
        targetVelocity = 0;
      } else {
        currentState = State.RAMPING_UP;
        stateTimer.reset();
      }
    }

    return targetVelocity;
  }

  public static double rpmToTicksPerSecond(double rpm) {
    return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
  }
}
