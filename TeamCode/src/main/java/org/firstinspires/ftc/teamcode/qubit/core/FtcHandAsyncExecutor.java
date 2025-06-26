package org.firstinspires.ftc.teamcode.qubit.core;

/**
 * A class to asynchronously operate the hand in the background.
 */
public class FtcHandAsyncExecutor implements Runnable {
  public enum HandOperation {
    Close,
    RotateDown,
    RotateUp,
    NoOp,
    Open,
    OpenToDeliver,
    Stop
  }

  public static final String TAG = "FtcHandAsyncExecutor";
  private final FtcHand hand;
  private final Object operationLock;
  private final Object statusLock;
  private HandOperation handOperation;
  private boolean handIsOpen = true;
  private boolean handIsUp = false;

  /**
   * Constructor.
   *
   * @param hand The hand object that actually does all the movements.
   */
  public FtcHandAsyncExecutor(FtcHand hand) {
    FtcLogger.enter();
    this.hand = hand;
    operationLock = new Object();
    statusLock = new Object();
    handOperation = HandOperation.NoOp;
    FtcLogger.exit();
  }

  public boolean handIsClosed() {
    boolean isClosed;
    synchronized (statusLock) {
      isClosed = !handIsOpen;
    }

    return isClosed;
  }

  public boolean handIsDown() {
    boolean isDown;
    synchronized (statusLock) {
      isDown = !handIsUp;
    }

    return isDown;
  }

  public boolean handIsOpen() {
    boolean isOpen;
    synchronized (statusLock) {
      isOpen = handIsOpen;
    }

    return isOpen;
  }

  public boolean handIsUp() {
    boolean isUp;
    synchronized (statusLock) {
      isUp = handIsUp;
    }

    return isUp;
  }

  /**
   * Sets the desired hand operation.
   *
   * @param handOperation      The hand operation to execute.
   * @param waitTillCompletion When True, waits will the operation is complete.
   */
  public void setOperation(HandOperation handOperation, boolean waitTillCompletion) {
    synchronized (operationLock) {
      this.handOperation = handOperation;
    }

    if (waitTillCompletion) {
      do {
        FtcUtils.sleep(5);
        synchronized (operationLock) {
          handOperation = this.handOperation;
        }
      } while (handOperation != HandOperation.NoOp);
    }
  }

  /**
   * The background execution method that runs in a tight loop.
   * This is independent of the robot execution loop.
   */
  public void run() {
    HandOperation requestedOperation;
    while (true) {
      // Get the latest requested operation.
      synchronized (operationLock) {
        requestedOperation = handOperation;
      }

      if (requestedOperation == HandOperation.Close) {
        hand.close(true);
        synchronized (statusLock) {
          handIsOpen = false;
        }

        synchronized (operationLock) {
          handOperation = HandOperation.NoOp;
        }
      } else if (requestedOperation == HandOperation.RotateDown) {
        hand.close(true);
        hand.rotateDown(true);
        synchronized (statusLock) {
          handIsOpen = false;
          handIsUp = false;
        }

        synchronized (operationLock) {
          handOperation = HandOperation.NoOp;
        }
      } else if (requestedOperation == HandOperation.RotateUp) {
        hand.rotateUp(true);
        synchronized (statusLock) {
          handIsUp = true;
        }

        synchronized (operationLock) {
          handOperation = HandOperation.NoOp;
        }
      } else if (requestedOperation == HandOperation.NoOp) {
        FtcUtils.sleep(5);
      } else if (requestedOperation == HandOperation.Open) {
        hand.open(true);
        synchronized (statusLock) {
          handIsOpen = true;
        }

        synchronized (operationLock) {
          handOperation = HandOperation.NoOp;
        }
      } else if (requestedOperation == HandOperation.OpenToDeliver) {
        hand.openToDeliver(true);
        synchronized (statusLock) {
          // hand is considered up and closed
          handIsUp = true;
          handIsOpen = false;
        }

        synchronized (operationLock) {
          handOperation = HandOperation.NoOp;
        }
      } else if (requestedOperation == HandOperation.Stop) {
        break;
      }
    }

    FtcLogger.info(TAG, "Async thread stopped.");
  }
}
