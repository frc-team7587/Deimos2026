package frc.robot.subsystems.floor;

import org.littletonrobotics.junction.AutoLog;

public interface FloorIO {
  @AutoLog
  public static class FloorIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(FloorIOInputs inputs) {}

  public default void setSpeed(double speed) {}

  public default void stop() {}
}
