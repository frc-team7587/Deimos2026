package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public Command feedCommand() {
    return Commands.runEnd(() -> io.setSpeed(FeederConstants.maxSpeed), io::stop, this);
  }
}
