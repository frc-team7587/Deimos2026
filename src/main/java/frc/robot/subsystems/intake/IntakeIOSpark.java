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
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOSpark implements IntakeIO {
  private final SparkFlex leftRoller =
      new SparkFlex(IntakeConstants.leftRollerCanId, MotorType.kBrushless);
  private final SparkFlex rightRoller =
      new SparkFlex(IntakeConstants.rightRollerCanId, MotorType.kBrushless);
  private final SparkMax leftPivot =
      new SparkMax(IntakeConstants.leftPivotCanId, MotorType.kBrushless);
  private final SparkMax rightPivot =
      new SparkMax(IntakeConstants.rightPivotCanId, MotorType.kBrushless);
  private final RelativeEncoder pivotEncoder = leftPivot.getEncoder();
  private final SparkClosedLoopController pivotController = leftPivot.getClosedLoopController();

  public IntakeIOSpark() {
    var leftRollerConfig = new SparkFlexConfig();
    leftRollerConfig.idleMode(IdleMode.kCoast).voltageCompensation(12.0);

    var rightRollerConfig = new SparkFlexConfig();
    rightRollerConfig
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12.0)
        .follow(IntakeConstants.leftRollerCanId, IntakeConstants.rightRollerInverted);

    var leftPivotConfig = new SparkMaxConfig();
    leftPivotConfig.idleMode(IdleMode.kBrake).voltageCompensation(12.0);
    leftPivotConfig.encoder
        .positionConversionFactor(IntakeConstants.pivotEncoderPositionFactor)
        .velocityConversionFactor(IntakeConstants.pivotEncoderVelocityFactor);
    leftPivotConfig.closedLoop
        .pid(IntakeConstants.pivotKp, IntakeConstants.pivotKi, IntakeConstants.pivotKd)
        .outputRange(-IntakeConstants.pivotMaxOutput, IntakeConstants.pivotMaxOutput);

    var rightPivotConfig = new SparkMaxConfig();
    rightPivotConfig
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0)
        .follow(IntakeConstants.leftPivotCanId, IntakeConstants.rightPivotInverted);

    leftRoller.configure(
        leftRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightRoller.configure(
        rightRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftPivot.configure(
        leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightPivot.configure(
        rightPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // The robot must start with the intake fully raised.
    pivotEncoder.setPosition(IntakeConstants.pivotMinAngleDegrees);
    setPivotPosition(IntakeConstants.pivotMinAngleDegrees);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.leftRollerConnected = leftRoller.getLastError() == REVLibError.kOk;
    inputs.rightRollerConnected = rightRoller.getLastError() == REVLibError.kOk;
    inputs.leftPivotConnected = leftPivot.getLastError() == REVLibError.kOk;
    inputs.rightPivotConnected = rightPivot.getLastError() == REVLibError.kOk;
    inputs.leftRollerAppliedVolts = leftRoller.getAppliedOutput() * leftRoller.getBusVoltage();
    inputs.rightRollerAppliedVolts = rightRoller.getAppliedOutput() * rightRoller.getBusVoltage();
    inputs.leftPivotAppliedVolts = leftPivot.getAppliedOutput() * leftPivot.getBusVoltage();
    inputs.rightPivotAppliedVolts = rightPivot.getAppliedOutput() * rightPivot.getBusVoltage();
    inputs.leftRollerCurrentAmps = leftRoller.getOutputCurrent();
    inputs.rightRollerCurrentAmps = rightRoller.getOutputCurrent();
    inputs.leftPivotCurrentAmps = leftPivot.getOutputCurrent();
    inputs.rightPivotCurrentAmps = rightPivot.getOutputCurrent();
    inputs.pivotPositionDegrees = pivotEncoder.getPosition();
    inputs.pivotVelocityDegreesPerSecond = pivotEncoder.getVelocity();
  }

  @Override
  public void setRollerSpeed(double speed) {
    leftRoller.set(speed);
  }

  @Override
  public void setPivotPosition(double positionDegrees) {
    pivotController.setSetpoint(positionDegrees, ControlType.kPosition);
  }

  @Override
  public void stopRollers() {
    leftRoller.stopMotor();
  }

}
