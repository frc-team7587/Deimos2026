package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leftConnected = false;
    public boolean rightConnected = false;
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double rightCurrentAmps = 0.0;
    public double leftVelocityRpm = 0.0;
    public double rightVelocityRpm = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Sets the signed shooter-wheel velocity in RPM. */
  public default void setVelocityRpm(double rpm) {}

  public default void stop() {}
}
