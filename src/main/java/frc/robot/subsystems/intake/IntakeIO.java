package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean leftRollerConnected = false;
    public boolean rightRollerConnected = false;
    public boolean leftPivotConnected = false;
    public boolean rightPivotConnected = false;
    public double leftRollerAppliedVolts = 0.0;
    public double rightRollerAppliedVolts = 0.0;
    public double leftPivotAppliedVolts = 0.0;
    public double rightPivotAppliedVolts = 0.0;
    public double leftRollerCurrentAmps = 0.0;
    public double rightRollerCurrentAmps = 0.0;
    public double leftPivotCurrentAmps = 0.0;
    public double rightPivotCurrentAmps = 0.0;
    public double pivotPositionDegrees = 0.0;
    public double pivotVelocityDegreesPerSecond = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerSpeed(double speed) {}

  public default void setPivotPosition(double positionDegrees) {}

  public default void stopRollers() {}

}
