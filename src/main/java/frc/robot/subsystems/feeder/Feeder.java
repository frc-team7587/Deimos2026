package frc.robot.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private double targetRpm = 0.0;

  public Feeder(FeederIO io) {
    this.io = io;
    SmartDashboard.putNumber("Feeder/DashboardTargetRpm", FeederConstants.defaultTargetRpm);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
    SmartDashboard.putNumber(
        "Feeder/MeasuredRpm", FeederConstants.feedingDirection * inputs.leftVelocityRpm);
    SmartDashboard.putNumber("Feeder/TargetRpm", targetRpm);
  }

  public Command feedCommand(BooleanSupplier useDashboardRpm) {
    return Commands.runEnd(
        () -> {
          double requestedRpm =
              useDashboardRpm.getAsBoolean()
                  ? SmartDashboard.getNumber(
                      "Feeder/DashboardTargetRpm", FeederConstants.defaultTargetRpm)
                  : FeederConstants.defaultTargetRpm;
          targetRpm =
              Double.isFinite(requestedRpm)
                  ? MathUtil.clamp(requestedRpm, 0.0, FeederConstants.maxTargetRpm)
                  : FeederConstants.defaultTargetRpm;
          io.setVelocityRpm(FeederConstants.feedingDirection * targetRpm);
        },
        () -> {
          io.stop();
          targetRpm = 0.0;
        },
        this);
  }
}
