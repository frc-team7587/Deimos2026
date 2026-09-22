package frc.robot.subsystems.intake;

public final class IntakeConstants {
  public static final int leftRollerCanId = 15;
  public static final int rightRollerCanId = 16;
  public static final int leftPivotCanId = 17;
  public static final int rightPivotCanId = 18;

  // Each pair is mounted on opposite ends of the same shaft.
  public static final boolean rightRollerInverted = true;
  public static final boolean rightPivotInverted = true;

  // Tune these after confirming the motors' installed directions.
  public static final double intakeSpeed = 1.0;
  public static final double outtakeSpeed = -1.0;

  // Pivot position configuration. Zero is the fully raised startup position.
  public static final double pivotReduction = 5.0 * 5.0;
  public static final double pivotEncoderPositionFactor = 360.0 / pivotReduction;
  public static final double pivotEncoderVelocityFactor = 360.0 / 60.0 / pivotReduction;
  public static final double pivotMinAngleDegrees = 0.0;
  public static final double pivotMaxAngleDegrees = 90.0;
  public static final double pivotAdjustmentRateDegreesPerSecond = 30.0;
  public static final double loopPeriodSeconds = 0.02;

  // Starting values for on-robot PID tuning.
  public static final double pivotKp = 0.02;
  public static final double pivotKi = 0.0;
  public static final double pivotKd = 0.0;
  public static final double pivotMaxOutput = 0.35;

  private IntakeConstants() {}
}
