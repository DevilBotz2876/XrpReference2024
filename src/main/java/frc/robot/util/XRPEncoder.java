// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.util;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import org.littletonrobotics.junction.Logger;

public class XRPEncoder extends Encoder {
  private static final double samplePeriod = 0.002;
  private static final double averagePeriod = 0.06;
  private static final int averageCount = (int) (averagePeriod / samplePeriod);
  private static final double slowPeriod = 0.02;

  private final Notifier fastNotifier;
  private final Notifier slowNotifier;
  private final CircularBuffer<TimestampedCount> fastBuffer = new CircularBuffer<>(averageCount);
  private final MedianFilter slowFilter = new MedianFilter(3);
  private Object fastVelocityLock = new Object();
  private Object slowVelocityLock = new Object();
  private double fastVelocity = 0.0;
  private double slowVelocity = 0.0;

  private static record TimestampedCount(double timestamp, int count) {}

  public XRPEncoder(int channelA, int channelB) {
    super(channelA, channelB);

    // Fast notifier: samples encoder and calculates raw velocity
    fastNotifier =
        new Notifier(
            () -> {
              double timestamp = Logger.getRealTimestamp() * 1.0e-6;
              int count = get();
              if (count == 0) return; // Not connected yet
              fastBuffer.addFirst(new TimestampedCount(timestamp, count));

              if (fastBuffer.size() < averageCount) return;
              var first = fastBuffer.getFirst();
              var last = fastBuffer.getLast();
              synchronized (fastVelocityLock) {
                fastVelocity =
                    (double) (first.count() - last.count())
                        / (first.timestamp() - last.timestamp());
              }
            });
    fastNotifier.setName("XRPEncoder[" + Integer.toString(channelA) + "]-Fast");
    fastNotifier.startPeriodic(samplePeriod);

    // Slow notifier: Applies median filter for every loop cycle to remove outliers
    slowNotifier =
        new Notifier(
            () -> {
              double velocity = 0.0;
              synchronized (fastVelocityLock) {
                velocity = slowFilter.calculate(fastVelocity);
              }
              synchronized (slowVelocityLock) {
                slowVelocity = velocity;
              }
            });
    slowNotifier.setName("XRPEncoder[" + Integer.toString(channelA) + "]-Slow");
    slowNotifier.startPeriodic(slowPeriod);
  }

  /**
   * Get the current rate of the encoder. Units are distance per second as scaled by the value from
   * setDistancePerPulse().
   *
   * @return The current rate of the encoder.
   */
  @Override
  public synchronized double getRate() {
    return slowVelocity * getDistancePerPulse();
  }
}