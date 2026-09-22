package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private double pivotTargetDegrees = IntakeConstants.pivotMinAngleDegrees;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.setPivotPosition(pivotTargetDegrees);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/PivotTargetDegrees", pivotTargetDegrees);
  }

  public Command intakeCommand() {
    return runRollersCommand(IntakeConstants.intakeSpeed);
  }

  public Command outtakeCommand() {
    return runRollersCommand(IntakeConstants.outtakeSpeed);
  }

  public Command pivotUpCommand() {
    return Commands.run(() -> adjustPivotTarget(-1.0), this);
  }

  public Command pivotDownCommand() {
    return Commands.run(() -> adjustPivotTarget(1.0), this);
  }

  private Command runRollersCommand(double speed) {
    return Commands.runEnd(() -> io.setRollerSpeed(speed), io::stopRollers, this);
  }

  private void adjustPivotTarget(double direction) {
    pivotTargetDegrees =
        MathUtil.clamp(
            pivotTargetDegrees
                + direction
                    * IntakeConstants.pivotAdjustmentRateDegreesPerSecond
                    * IntakeConstants.loopPeriodSeconds,
            IntakeConstants.pivotMinAngleDegrees,
            IntakeConstants.pivotMaxAngleDegrees);
  }
}
