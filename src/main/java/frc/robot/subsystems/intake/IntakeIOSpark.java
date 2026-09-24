package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Configs.IntakeConfig;

public class IntakeIOSpark implements IntakeIO {
  private final SparkFlex leftRoller =
      new SparkFlex(IntakeConstants.leftRollerCanId, MotorType.kBrushless);
  private final SparkFlex rightRoller =
      new SparkFlex(IntakeConstants.rightRollerCanId, MotorType.kBrushless);
  private final SparkMax pivotLeaderMotor;
  private final SparkMax pivotFollowerMotor;

  private final RelativeEncoder pivotLeaderEncoder;
  private final SparkClosedLoopController pivotLeaderController;

  public IntakeIOSpark() {
    var leftRollerConfig = new SparkFlexConfig();
    leftRollerConfig.idleMode(IdleMode.kCoast).voltageCompensation(12.0);

    var rightRollerConfig = new SparkFlexConfig();
    rightRollerConfig
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12.0)
        .follow(IntakeConstants.leftRollerCanId, IntakeConstants.rightRollerInverted);

    leftRoller.configure(
        leftRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightRoller.configure(
        rightRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotLeaderMotor = new SparkMax(IntakeConstants.kLeaderID, MotorType.kBrushless);
    pivotFollowerMotor = new SparkMax(IntakeConstants.kFollowerID, MotorType.kBrushless);

    pivotLeaderEncoder = pivotLeaderMotor.getEncoder();
    pivotLeaderController = pivotLeaderMotor.getClosedLoopController();

    pivotLeaderMotor.configure(
        IntakeConfig.pivotMotorLeaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    pivotFollowerMotor.configure(
        IntakeConfig.pivotMotorFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.leftRollerConnected = leftRoller.getLastError() == REVLibError.kOk;
    inputs.rightRollerConnected = rightRoller.getLastError() == REVLibError.kOk;
    inputs.leftRollerAppliedVolts = leftRoller.getAppliedOutput() * leftRoller.getBusVoltage();
    inputs.rightRollerAppliedVolts = rightRoller.getAppliedOutput() * rightRoller.getBusVoltage();
    inputs.leftRollerCurrentAmps = leftRoller.getOutputCurrent();
    inputs.rightRollerCurrentAmps = rightRoller.getOutputCurrent();
    inputs.pivotPosition = getPivotPosition();
  }

  @Override
  public void setRollerSpeed(double speed) {
    leftRoller.set(speed);
  }

  @Override
  public void stopRollers() {
    leftRoller.stopMotor();
  }

  @Override
  public void setPivotSpeed(double speed) {
    pivotLeaderMotor.set(speed);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotLeaderMotor.setVoltage(volts);
  }

  @Override
  public void setPivotPosition(double position) {
    pivotLeaderController.setSetpoint(position, ControlType.kPosition);
  }

  @Override
  public double getPivotPosition() {
    return pivotLeaderEncoder.getPosition();
  }
}
