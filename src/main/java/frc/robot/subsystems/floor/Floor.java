package frc.robot.subsystems.floor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Floor extends SubsystemBase {
  private final FloorIO io;
  private final FloorIOInputsAutoLogged inputs = new FloorIOInputsAutoLogged();

  public Floor(FloorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Floor", inputs);
  }

  public Command forwardCommand() {
    return runAtSpeedCommand(FloorConstants.forwardSpeed);
  }

  public Command reverseCommand() {
    return runAtSpeedCommand(FloorConstants.reverseSpeed);
  }

  public Command speedCommand(DoubleSupplier speedSupplier) {
    return Commands.runEnd(() -> io.setSpeed(speedSupplier.getAsDouble()), io::stop, this);
  }

  private Command runAtSpeedCommand(double speed) {
    return Commands.runEnd(() -> io.setSpeed(speed), io::stop, this);
  }
}
