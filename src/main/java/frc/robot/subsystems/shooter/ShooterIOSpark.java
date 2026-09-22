package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOSpark implements ShooterIO {
  private final SparkMax leftMotor = new SparkMax(ShooterConstants.leftCanId, MotorType.kBrushless);
  private final SparkMax rightMotor =
      new SparkMax(ShooterConstants.rightCanId, MotorType.kBrushless);

  public ShooterIOSpark() {
    var leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kCoast).voltageCompensation(12.0);

    var rightConfig = new SparkMaxConfig();
    rightConfig
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12.0)
        .follow(ShooterConstants.leftCanId, true);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
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
