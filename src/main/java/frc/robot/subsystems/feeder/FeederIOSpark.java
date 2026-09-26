package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FeederIOSpark implements FeederIO {
  private final SparkMax leftMotor = new SparkMax(FeederConstants.leftCanId, MotorType.kBrushless);
  private final SparkMax rightMotor =
      new SparkMax(FeederConstants.rightCanId, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private final SparkClosedLoopController velocityController = leftMotor.getClosedLoopController();
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          FeederConstants.velocityKsVolts, FeederConstants.velocityKvVoltsPerRpm);

  public FeederIOSpark() {
    var leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kCoast).voltageCompensation(12.0);
    leftConfig
        .encoder
        .positionConversionFactor(1.0 / FeederConstants.gearReduction)
        .velocityConversionFactor(1.0 / FeederConstants.gearReduction);
    leftConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(FeederConstants.velocityKp, FeederConstants.velocityKi, FeederConstants.velocityKd)
        .outputRange(-1.0, 1.0);

    var rightConfig = new SparkMaxConfig();
    rightConfig.apply(leftConfig).follow(FeederConstants.leftCanId, true);

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
    inputs.leftVelocityRpm = leftEncoder.getVelocity();
    inputs.rightVelocityRpm = rightEncoder.getVelocity();
  }

  @Override
  public void setVelocityRpm(double rpm) {
    velocityController.setSetpoint(
        rpm,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(rpm),
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
  }
}
