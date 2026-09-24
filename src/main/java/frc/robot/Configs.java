package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.intake.IntakePivot.IntakePivotConstants;

public class Configs {
  public static final class IntakeConfig {
    public static final SparkMaxConfig pivotMotorLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig pivotMotorFollowerConfig = new SparkMaxConfig();
    public static final SoftLimitConfig intakeSoftLimit = new SoftLimitConfig();

    static {
      pivotMotorLeaderConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
      pivotMotorLeaderConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(
              IntakePivotConstants.kP,
              IntakePivotConstants.kI,
              IntakePivotConstants.kD,
              IntakePivotConstants.kFF)
          .outputRange(IntakePivotConstants.kMinOutput, IntakePivotConstants.kMaxOutput);
      pivotMotorFollowerConfig
          .apply(pivotMotorLeaderConfig)
          .follow(IntakePivotConstants.kLeaderID, true);
    }
  }
}
