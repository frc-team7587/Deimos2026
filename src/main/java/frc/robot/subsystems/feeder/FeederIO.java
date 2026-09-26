package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public boolean leftConnected = false;
    public boolean rightConnected = false;
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double rightCurrentAmps = 0.0;
    public double leftVelocityRpm = 0.0;
    public double rightVelocityRpm = 0.0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  /** Sets the signed feeder-output velocity in RPM. */
  public default void setVelocityRpm(double rpm) {}

  public default void stop() {}
}
