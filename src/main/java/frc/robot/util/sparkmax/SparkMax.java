/* Copyright 2023 Pike RoboDevils, FRC Team 1018
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE.md file or
 * at https://opensource.org/licenses/MIT. */

package frc.robot.util.sparkmax;

/*
 * Adapted from 3005's 2022 Code
 * Original source published at https://github.com/FRC3005/Rapid-React-2022-Public/tree/d499655448ed592c85f9cfbbd78336d8841f46e2
 */

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

public class SparkMax extends CANSparkMax {

  /**
   * Store a reference to every spark max.
   *
   * <p>TODO: Release the reference if it is closed.
   */
  private static List<SparkMax> m_sparkMaxes = new ArrayList<>();

  /**
   * Create a motor controller from a CANSparkMax object.
   *
   * @param sparkMax A CANSparkMax object. Run any initialization that you want excluded from the
   *     built in health monitor functions
   * @param initFunction A function which takes a CANSparkMax and returns a Boolean. This function
   *     is used to initialize the CANSparkMax device, and is called in one of two places. 1) It is
   *     called in this constructor, and 2) it is called in the case of a health monitor timeout
   *     (i.e. the controller has reset)
   */
  public SparkMax(int canId, MotorType motorType) {
    super(canId, motorType);

    // Always start fresh and apply settings in code for each device
    // Add delay to avoid any possible timing issues.
    Timer.delay(0.050);

    // If a parameter set fails, this will add more time to alleviate any bus
    // traffic
    // default is 20ms
    setCANTimeout(0);
    m_sparkMaxes.add(this);
  }

  /**
   * Create a SPARK MAX motor controller with a brushless motor
   *
   * @param canId Spark Max CAN Id
   */
  public SparkMax(int canId) {
    this(canId, MotorType.kBrushless);
  }

  /**
   * Run burnFlash() for all controllers initialized. The ideal use case for this call is to call it
   * once everything has been initialized. The burnFlash() call has the side effect of preventing
   * all communication *to* the device for up to 200ms or more, potentially including some messages
   * called before the burnFlash() call, and receiveing messages *from* the device.
   *
   * <p>WARNING: This call will sleep the thread before and after burning flash. This is for your
   * safety.
   */
  public static void burnFlashInSync() {
    // DriverStation.reportWarning(
    // 		String.format("Burning Flash Count: {}", ++m_burnFlashCnt), false);
    // Logger.tag("SparkMax").debug("Burning Flash Count: {}", ++m_burnFlashCnt);
    Timer.delay(0.25);
    for (SparkMax max : m_sparkMaxes) {
      // DriverStation.reportWarning(
      // 		String.format("Burning Flash Count for Can ID {}", max.getDeviceId()), false);
      // Logger.tag("SparkMax").trace("Burning flash for Can ID {}",
      // max.getDeviceId());
      max.burnFlash();
      // Enough time to not spam the bus too bad
      Timer.delay(0.005);
    }
    Timer.delay(0.25);
    DriverStation.reportWarning("Burn Flash Complete", false);
    // Logger.tag("SparkMax").debug("Burn Flash Complete.");
  }
}
