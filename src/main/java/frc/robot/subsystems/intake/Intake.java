package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Subsystem rollerRequirement = new Subsystem() {};
  private final Subsystem pivotRequirement = new Subsystem() {};
  private final SysIdRoutine pivotSysId;

  private final LoggedMechanism2d mechPanel;
  private final LoggedMechanismRoot2d mechRoot;
  private final LoggedMechanismLigament2d mechArm;
  private final LoggedMechanismRoot2d mechIntakeRoot;
  private final LoggedMechanismLigament2d mechIntake;

  public Intake(IntakeIO io) {
    this.io = io;

    SmartDashboard.putNumber("Intake/PivotPosition", io.getPivotPosition());

    pivotSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/PivotSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setPivotVoltage(voltage.in(Volts)), null, pivotRequirement));

    mechPanel =
        new LoggedMechanism2d(
            Units.inchesToMeters(100),
            Units.inchesToMeters(100),
            new Color8Bit(Color.kGray)); // view panel size
    mechRoot = mechPanel.getRoot("root", Units.inchesToMeters(50), Units.inchesToMeters(0));
    mechArm =
        mechRoot.append(
            new LoggedMechanismLigament2d("arm", Units.inchesToMeters(8), 90, 10, new Color8Bit()));
    mechIntakeRoot =
        mechPanel.getRoot("intakeroot", Units.inchesToMeters(50), Units.inchesToMeters(58));
    mechIntake =
        mechArm.append(
            new LoggedMechanismLigament2d(
                "intake", Units.inchesToMeters(24), 0, 10, new Color8Bit(Color.kRed)));
  }

  public Command setPivotPosition(double position) {
    return Commands.run(() -> io.setPivotPosition(position), pivotRequirement);
  }

  public Command intakeCommand() {
    return runRollersCommand(IntakeConstants.intakeSpeed);
  }

  public Command outtakeCommand() {
    return runRollersCommand(IntakeConstants.outtakeSpeed);
  }

  public Command pivotUpCommand() {
    SmartDashboard.putNumber("Intake/PivotPosition", io.getPivotPosition());

    return Commands.startEnd(
        () -> io.setPivotSpeed(IntakeConstants.kPivotSpeedUp),
        () -> io.setPivotPosition(io.getPivotPosition()),
        pivotRequirement);
  }

  public Command pivotDownCommand() {
    SmartDashboard.putNumber("Intake/PivotPosition", io.getPivotPosition());

    return Commands.startEnd(
        () -> io.setPivotSpeed(IntakeConstants.kPivotSpeedDown),
        () -> io.setPivotPosition(io.getPivotPosition()),
        pivotRequirement);
  }

  private Command runRollersCommand(double speed) {
    return Commands.runEnd(() -> io.setRollerSpeed(speed), io::stopRollers, rollerRequirement);
  }

  public Command pivotSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return pivotSysId.quasistatic(direction);
  }

  public Command pivotSysIdDynamic(SysIdRoutine.Direction direction) {
    return pivotSysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("intakeMech", mechPanel);
    // turns encoder position to degrees
    mechIntake.setAngle(new Rotation2d(inputs.pivotPosition - 90.0));
  }
}
