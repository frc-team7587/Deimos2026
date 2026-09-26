package frc.robot.subsystems.shooter;

public final class ShooterConstants {
  public static final int leftCanId = 9;
  public static final int rightCanId = 14;

  public static final double gearReduction = 1.0;
  public static final double defaultTargetRpm = 4200.0;
  public static final double maxTargetRpm = 5500.0;
  public static final double shootingDirection = -1.0;

  // Larry's PID gains; feedforward velocity gain is adjusted for Deimos's 1:1 gearing.
  public static final double velocityKp = 0.00008;
  public static final double velocityKi = 0.0;
  public static final double velocityKd = 0.0;
  public static final double velocityKsVolts = 0.7325;
  public static final double velocityKvVoltsPerRadPerSec = 0.057 * gearReduction / 3.0;

  private ShooterConstants() {}
}
