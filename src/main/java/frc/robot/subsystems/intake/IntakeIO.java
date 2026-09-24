package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public boolean leftRollerConnected = false;
    public boolean rightRollerConnected = false;
    public double leftRollerAppliedVolts = 0.0;
    public double rightRollerAppliedVolts = 0.0;
    public double leftRollerCurrentAmps = 0.0;
    public double rightRollerCurrentAmps = 0.0;
    public double pivotPosition = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setRollerSpeed(double speed) {}

  default void stopRollers() {}

  /**
   * Sets the speed of the pivot motors.
   *
   * @param speed The speed to set the pivot motors to.
   */
  default void setPivotSpeed(double speed) {}

  /** Sets pivot motor with direct voltage for characterization. */
  default void setPivotVoltage(double volts) {}

  /**
   * Sets the position of the pivot motors.
   *
   * @param position The position to set the pivot motors to.
   */
  default void setPivotPosition(double position) {}

  /**
   * Gets the encoder value of the pivot motors.
   *
   * @return The encoder value of the pivot motors.
   */
  default double getPivotPosition() {
    return 0.0;
  }
}
