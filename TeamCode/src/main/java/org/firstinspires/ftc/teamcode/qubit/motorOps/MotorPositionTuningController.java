package org.firstinspires.ftc.teamcode.qubit.motorOps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class MotorPositionTuningController {
  public static int TESTING_MIN_POSITION = 0;
  public static int TESTING_MAX_POSITION = 2000;
  public static int POSITION_TEST_1 = 500;
  public static int POSITION_TEST_2 = 1500;
  public static int POSITION_TEST_3 = 1000;

  // State durations - holding times at each position
  // Prefixed with "ZSTATE" to appear below other settings in Dashboard (alphabetical order)
  public static double ZSTATE1_HOLD_MIN_DURATION = 2;
  public static double ZSTATE2_HOLD_MAX_DURATION = 2;
  public static double ZSTATE3_HOLD_TEST1_DURATION = 2;
  public static double ZSTATE4_HOLD_TEST2_DURATION = 2;
  public static double ZSTATE5_HOLD_TEST3_DURATION = 2;
  public static double ZSTATE6_REST_DURATION = 1;

  enum State {
    MOVE_TO_MIN,
    HOLD_MIN,
    MOVE_TO_MAX,
    HOLD_MAX,
    MOVE_TO_TEST1,
    HOLD_TEST1,
    MOVE_TO_TEST2,
    HOLD_TEST2,
    MOVE_TO_TEST3,
    HOLD_TEST3,
    REST
  }

  private final ElapsedTime stateTimer = new ElapsedTime();
  private State currentState = State.REST;
  private int targetPosition = 0;

  public void start() {
    stateTimer.reset();
    currentState = State.REST;
    targetPosition = 0;
  }

  public int getTargetPosition() {
    if (currentState == State.MOVE_TO_MIN) {
      targetPosition = TESTING_MIN_POSITION;
      currentState = State.HOLD_MIN;
      stateTimer.reset();
    } else if (currentState == State.HOLD_MIN) {
      if (stateTimer.seconds() > ZSTATE1_HOLD_MIN_DURATION) {
        currentState = State.MOVE_TO_MAX;
        stateTimer.reset();
      }
    } else if (currentState == State.MOVE_TO_MAX) {
      targetPosition = TESTING_MAX_POSITION;
      currentState = State.HOLD_MAX;
      stateTimer.reset();
    } else if (currentState == State.HOLD_MAX) {
      if (stateTimer.seconds() > ZSTATE2_HOLD_MAX_DURATION) {
        currentState = State.MOVE_TO_TEST1;
        stateTimer.reset();
      }
    } else if (currentState == State.MOVE_TO_TEST1) {
      targetPosition = POSITION_TEST_1;
      currentState = State.HOLD_TEST1;
      stateTimer.reset();
    } else if (currentState == State.HOLD_TEST1) {
      if (stateTimer.seconds() > ZSTATE3_HOLD_TEST1_DURATION) {
        currentState = State.MOVE_TO_TEST2;
        stateTimer.reset();
      }
    } else if (currentState == State.MOVE_TO_TEST2) {
      targetPosition = POSITION_TEST_2;
      currentState = State.HOLD_TEST2;
      stateTimer.reset();
    } else if (currentState == State.HOLD_TEST2) {
      if (stateTimer.seconds() > ZSTATE4_HOLD_TEST2_DURATION) {
        currentState = State.MOVE_TO_TEST3;
        stateTimer.reset();
      }
    } else if (currentState == State.MOVE_TO_TEST3) {
      targetPosition = POSITION_TEST_3;
      currentState = State.HOLD_TEST3;
      stateTimer.reset();
    } else if (currentState == State.HOLD_TEST3) {
      if (stateTimer.seconds() > ZSTATE5_HOLD_TEST3_DURATION) {
        currentState = State.REST;
        stateTimer.reset();
      }
    } else if (currentState == State.REST) {
      targetPosition = 0;
      if (stateTimer.seconds() > ZSTATE6_REST_DURATION) {
        currentState = State.MOVE_TO_MIN;
        stateTimer.reset();
      }
    }

    return targetPosition;
  }

  public State getCurrentState() {
    return currentState;
  }
}
