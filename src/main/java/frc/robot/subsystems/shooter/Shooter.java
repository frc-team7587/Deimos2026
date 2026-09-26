package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private double targetRpm = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
    SmartDashboard.putNumber("Shooter/DashboardTargetRpm", ShooterConstants.defaultTargetRpm);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    SmartDashboard.putNumber(
        "Shooter/MeasuredRpm", ShooterConstants.shootingDirection * inputs.leftVelocityRpm);
    SmartDashboard.putNumber("Shooter/TargetRpm", targetRpm);
  }

  public Command shootCommand(BooleanSupplier useDashboardRpm) {
    return Commands.runEnd(
        () -> {
          double requestedRpm =
              useDashboardRpm.getAsBoolean()
                  ? SmartDashboard.getNumber(
                      "Shooter/DashboardTargetRpm", ShooterConstants.defaultTargetRpm)
                  : ShooterConstants.defaultTargetRpm;
          targetRpm =
              Double.isFinite(requestedRpm)
                  ? MathUtil.clamp(requestedRpm, 0.0, ShooterConstants.maxTargetRpm)
                  : ShooterConstants.defaultTargetRpm;
          io.setVelocityRpm(ShooterConstants.shootingDirection * targetRpm);
        },
        () -> {
          io.stop();
          targetRpm = 0.0;
        },
        this);
  }

  public Command shootCommandTypeShi(BooleanSupplier useDashboardRpm, double inertiaty) {
    return Commands.runEnd(
        () -> {
          double requestedRpm = 3000 + 1500 * inertiaty;
          targetRpm =
              Double.isFinite(requestedRpm)
                  ? MathUtil.clamp(requestedRpm, 0.0, ShooterConstants.maxTargetRpm)
                  : ShooterConstants.defaultTargetRpm;
          io.setVelocityRpm(ShooterConstants.shootingDirection * targetRpm);
        },
        () -> {
          io.stop();
          targetRpm = 0.0;
        },
        this);
  }
}
