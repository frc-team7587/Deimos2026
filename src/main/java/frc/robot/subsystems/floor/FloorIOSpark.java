package frc.robot.subsystems.floor;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class FloorIOSpark implements FloorIO {
  private final SparkMax motor = new SparkMax(FloorConstants.motorCanId, MotorType.kBrushless);

  public FloorIOSpark() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).voltageCompensation(12.0);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(FloorIOInputs inputs) {
    inputs.connected = motor.getLastError() == REVLibError.kOk;
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
