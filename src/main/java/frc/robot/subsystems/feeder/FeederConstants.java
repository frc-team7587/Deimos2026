package frc.robot.subsystems.feeder;

public final class FeederConstants {
  public static final int leftCanId = 10;
  public static final int rightCanId = 15;

  public static final double gearReduction = 4.0;
  public static final double defaultTargetRpm = 1250.0;
  public static final double maxTargetRpm = 1400.0;
  public static final double feedingDirection = -1.0;

  // Larry's PID gains; feedforward velocity gain is adjusted for Deimos's 4:1 gearing.
  public static final double velocityKp = 0.001;
  public static final double velocityKi = 0.0;
  public static final double velocityKd = 0.0;
  public static final double velocityKsVolts = 0.9175;
  public static final double velocityKvVoltsPerRpm = 0.00995 * gearReduction / 5.0;

  private FeederConstants() {}
}
