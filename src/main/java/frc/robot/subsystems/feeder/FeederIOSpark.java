package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class FeederIOSpark implements FeederIO {
  private final SparkMax leftMotor = new SparkMax(FeederConstants.leftCanId, MotorType.kBrushless);
  private final SparkMax rightMotor =
      new SparkMax(FeederConstants.rightCanId, MotorType.kBrushless);

  public FeederIOSpark() {
    var leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kCoast).voltageCompensation(12.0);

    var rightConfig = new SparkMaxConfig();
    rightConfig
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12.0)
        .follow(FeederConstants.leftCanId, true);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.leftConnected = leftMotor.getLastError() == REVLibError.kOk;
    inputs.rightConnected = rightMotor.getLastError() == REVLibError.kOk;
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
    inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
  }

  @Override
  public void setSpeed(double speed) {
    leftMotor.set(speed);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
  }
}
