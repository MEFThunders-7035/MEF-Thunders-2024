package wrapper_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.utils.sim_utils.CANSparkMAXWrapped;
import frc.utils.sim_utils.SparkMAXSimAddon;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SparkMAXWrapperTest {
  @BeforeEach
  public void setUp() {
    HAL.initialize(500, 0); // init HAL just in case
  }

  @Test
  void testSparkMAXWrapper() {
    for (int i = 0; i < 50; i++) {
      CANSparkMAXWrapped sparkMAX =
          new CANSparkMAXWrapped(i, CANSparkMAXWrapped.MotorType.kBrushless);
      assertEquals(sparkMAX, SparkMAXSimAddon.getSparkMAX(i));
      sparkMAX.close();
    }
  }

  @Test
  void testSparkMAXisClosed() {
    CANSparkMAXWrapped sparkMAX =
        new CANSparkMAXWrapped(1, CANSparkMAXWrapped.MotorType.kBrushless);
    sparkMAX.close();
    assertEquals(true, sparkMAX.isThisClosed());
  }

  @Test
  void testSparkMAXClear() {
    CANSparkMAXWrapped sparkMAX =
        new CANSparkMAXWrapped(1, CANSparkMAXWrapped.MotorType.kBrushless);
    SparkMAXSimAddon.resetData();
    try {
      SparkMAXSimAddon.getSparkMAX(1);
      assert false; // should not reach here
      sparkMAX.close(); // just so the linter doesn't complain
    } catch (IllegalArgumentException e) {
      assertEquals("SparkMAX with deviceID 1 does not exist", e.getMessage());
    }
  }
}
