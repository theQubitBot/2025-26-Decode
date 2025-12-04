package org.firstinspires.ftc.teamcode.qubit.core;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.opencv.core.Scalar;

import java.lang.reflect.Modifier;
import java.util.concurrent.TimeUnit;

/**
 * Global utility functions
 */
public final class FtcUtils {
  public static final String TAG = ">";
  public static final boolean DEBUG = false;
  public static final double EPSILON0 = 0;
  public static final double EPSILON1 = 1e-1;
  public static final double EPSILON2 = 1e-2;
  public static final double EPSILON3 = 1e-3;
  public static final double EPSILON4 = 1e-4;
  public static final int AUTO_OP_DURATION = 30; // seconds
  public static final int AUTO_2_TELE_OP_TRANSITION_TIME = 8; // seconds
  public static final int TELE_OP_DURATION = 120; // seconds
  public static final int END_GAME_DURATION = 30; // seconds
  public static final int BUZZER_DURATION = 2; // seconds
  public static final int CYCLE_MS = 50; // milliseconds
  public static final int INTERRUPTIBLE_CYCLE_MS = 5; // milliseconds

  // Gamepad thresholds
  public static final double TRIGGER_THRESHOLD = 0.5;

  // Timing constants for endgame warnings
  public static final int ENDGAME_PARK_WARNING_SECONDS = 10;

  /* Constructor */
  public FtcUtils() {
  }

  public static Scalar add(Scalar a, Scalar b) {
    return new Scalar(a.val[0] + b.val[0],
        a.val[1] + b.val[1],
        a.val[2] + b.val[2],
        a.val[3] + b.val[3]);
  }

  public static Scalar average(Scalar a, Scalar b) {
    return new Scalar((a.val[0] + b.val[0]) / 2.0,
        (a.val[1] + b.val[1]) / 2.0,
        (a.val[2] + b.val[2]) / 2.0,
        (a.val[3] + b.val[3]) / 2.0);
  }

  /**
   * Determines if a and b are very close to each other.
   *
   * @param a The first number.
   * @param b The second number.
   * @return True, if a and b are less than EPSILON apart.
   */
  public static boolean areEqual(double a, double b, double epsilon) {
    return Math.abs(a - b) <= epsilon;
  }

  /**
   * Determines if a and b are very close to each other.
   *
   * @param a The first number.
   * @param b The second number.
   * @return True, if a and b are less than EPSILON apart.
   */
  public static boolean areEqual(float a, float b, double epsilon) {
    return Math.abs(a - b) <= epsilon;
  }

  public static boolean areEqual(int a, int b, int epsilon) {
    return Math.abs(a - b) <= epsilon;
  }

  public static boolean lastNSeconds(ElapsedTime runtime, int nSeconds) {
    return runtime.seconds() <= TELE_OP_DURATION && runtime.seconds() >= (TELE_OP_DURATION - nSeconds);
  }

  public static boolean endGame(ElapsedTime runtime) {
    return lastNSeconds(runtime, END_GAME_DURATION);
  }

  public static boolean gameOver(ElapsedTime runtime) {
    // Include buzzer duration in game play time.
    return runtime.seconds() >= (TELE_OP_DURATION + BUZZER_DURATION);
  }

  /**
   * Determines if subsystems should stop operating (game over in non-debug mode).
   *
   * @param runtime The tele op runtime.
   * @return true if subsystems should stop, false otherwise.
   */
  public static boolean shouldStopOperating(ElapsedTime runtime) {
    return !DEBUG && gameOver(runtime);
  }

  /**
   * Determines if a given value lies in a given range.
   *
   * @param value    The value to evaluate
   * @param minValue The minimum (inclusive) value
   * @param maxValue The maximum (inclusive) value
   * @return True if the given value is in the range [minValue, maxValue], false otherwise
   */
  public static boolean inRange(double value, double minValue, double maxValue) {
    return value >= minValue && value <= maxValue;
  }

  /**
   * Determines if a given value lies outside a given range.
   *
   * @param value    The value to evaluate
   * @param minValue The minimum value
   * @param maxValue The maximum value
   * @return True if the given value is outside the range [minValue, maxValue], false otherwise
   */
  public static boolean outsideRange(double value, double minValue, double maxValue) {
    return value < minValue || value > maxValue;
  }

  /**
   * Determines if a given values lies in a given array of values.
   *
   * @param array The value to evaluate.
   * @param value The array of values to use.
   * @return True if the given values is in the array of value, false otherwise.
   */
  public static boolean contains(int[] array, int value) {
    Assert.assertNotNull(array, "contains>array");
    for (int i : array) {
      if (i == value) return true;
    }

    return false;
  }

  /**
   * Sleep until given deadline expires.
   *
   * @param deadline The remaining duration to sleep until.
   */
  public static void sleep(Deadline deadline) {
    if (deadline != null && !deadline.hasExpired()) {
      sleep(deadline.timeRemaining(TimeUnit.MILLISECONDS));
    }
  }

  /**
   * Sleep for given milli seconds.
   *
   * @param milliseconds The time to sleep in milliseconds.
   */
  public static void interruptibleSleep(long milliseconds, LinearOpMode autoOpMode) {
    try {
      if (milliseconds >= 0) {
        Deadline d = new Deadline(milliseconds, TimeUnit.MILLISECONDS);
        while (!d.hasExpired() && autoOpMode.opModeIsActive()) {
          Thread.sleep(INTERRUPTIBLE_CYCLE_MS);
        }
      }
    } catch (InterruptedException ignored) {
    }
  }

  /**
   * Sleep for given milli seconds.
   *
   * @param milliseconds The time to sleep in milliseconds.
   */
  public static void sleep(long milliseconds) {
    try {
      if (milliseconds >= 0) {
        Thread.sleep(milliseconds);
      }
    } catch (InterruptedException ignored) {
    }
  }

  /**
   * Serializes an object into its JSon string representation.
   *
   * @param o The object to serialize.
   * @return The JSon string representation of the input object.
   */
  public static String serialize(Object o) {
    GsonBuilder gsonBuilder = new GsonBuilder();

    // Allow serialization of public fields only
    gsonBuilder.excludeFieldsWithModifiers(Modifier.ABSTRACT, Modifier.FINAL, Modifier.INTERFACE,
        Modifier.PRIVATE, Modifier.PROTECTED, Modifier.STATIC,
        Modifier.TRANSIENT, Modifier.VOLATILE);

    // Creates a Gson instance based on the current configuration
    Gson gson = gsonBuilder.create();
    return gson.toJson(o);
  }

  /**
   * Deserializes the given JSon string into an object of given type.
   *
   * @param data    The JSon string representation of the object.
   * @param classOf The class of the object to deserialize to.
   * @param <T>     The type of object.
   * @return The deserialized object.
   */
  public static <T> T deserialize(String data, Class<T> classOf) {
    return SimpleGson.getInstance().fromJson(data, classOf);
  }
}
