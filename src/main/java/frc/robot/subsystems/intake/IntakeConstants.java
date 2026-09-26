package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final int leftRollerCanId = 13;
  public static final int rightRollerCanId = 17;
  public static final int kLeaderID = 11;
  public static final int kFollowerID = 16;

  public static final boolean rightRollerInverted = true;
  public static final double intakeSpeed = 0.3;
  public static final double outtakeSpeed = -0.3;

  public static final double kPivotSpeedDown = 0.26;
  public static final double kPivotSpeedUp = -0.6;

  public static final double kP = 0.07;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kFF = 0.00375;

  public static final double kMinOutput = -0.4;
  public static final double kMaxOutput = 0.4;

  private IntakeConstants() {}
}
