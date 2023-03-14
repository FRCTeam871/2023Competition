package com.team871.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class ArmExtensionTest {
  // To learn more about how to write unit tests, see the
  // JUnit 5 User Guide at https://junit.org/junit5/docs/current/user-guide/

  @Test
  void testSafetyRamp() {
    // assertEquals(-1, ArmExtension.limitOutput(-1, 4));
    // assertEquals(-.3, ArmExtension.limitOutput(-1, 0));
    // assertEquals(-.5, ArmExtension.limitOutput(-1, 1.5));

    // assertEquals(-.3, ArmExtension.limitOutput(-.3, 1.5));
    // assertEquals(-.5/3, ArmExtension.limitOutput(-.3, .5));

    assertEquals(1, ArmExtension.limitOutput(1, 16));
    assertEquals(.5, ArmExtension.limitOutput(1, 17.5));
    assertEquals(.3, ArmExtension.limitOutput(.3, 17.5));
    assertEquals(.5 / 3, ArmExtension.limitOutput(1, 18.5));
  }
}
