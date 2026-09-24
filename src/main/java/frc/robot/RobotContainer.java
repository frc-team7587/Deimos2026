// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSpark;
import frc.robot.subsystems.floor.Floor;
import frc.robot.subsystems.floor.FloorConstants;
import frc.robot.subsystems.floor.FloorIO;
import frc.robot.subsystems.floor.FloorIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Feeder feeder;
  private final Shooter shooter;
  private final Floor floor;
  private final Intake intake;
  private boolean floorReverseHeld = false;
  private boolean floorForwardHeld = false;
  private boolean floorReversePressedMostRecently = false;
  public static boolean robotRelative = true;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private static final double driverTurnScale = 0.7;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        feeder = new Feeder(new FeederIOSpark());
        shooter = new Shooter(new ShooterIOSpark());
        floor = new Floor(new FloorIOSpark());
        intake = new Intake(new IntakeIOSpark());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        feeder = new Feeder(new FeederIO() {});
        shooter = new Shooter(new ShooterIO() {});
        floor = new Floor(new FloorIO() {});
        intake = new Intake(new IntakeIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        feeder = new Feeder(new FeederIO() {});
        shooter = new Shooter(new ShooterIO() {});
        floor = new Floor(new FloorIO() {});
        intake = new Intake(new IntakeIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -MathUtil.applyDeadband(controller.getLeftY(), 0.05),
            () -> -MathUtil.applyDeadband(controller.getLeftX(), 0.05),
            () -> -MathUtil.applyDeadband(driverTurnScale * controller.getRightX(), 0.05)));

    // change from robot relative to field relativ e
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  robotRelative = !robotRelative;
                }));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                shooter.shootCommand(),
                Commands.waitSeconds(1.0).andThen(feeder.feedCommand())));

    controller
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  floorReverseHeld = true;
                  floorReversePressedMostRecently = true;
                }))
        .onFalse(Commands.runOnce(() -> floorReverseHeld = false));
    controller
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  floorForwardHeld = true;
                  floorReversePressedMostRecently = false;
                }))
        .onFalse(Commands.runOnce(() -> floorForwardHeld = false));
    floor.setDefaultCommand(
        floor.speedCommand(
            () -> {
              if (floorReverseHeld && floorForwardHeld) {
                return floorReversePressedMostRecently
                    ? FloorConstants.reverseSpeed
                    : FloorConstants.forwardSpeed;
              }
              if (floorReverseHeld) {
                return FloorConstants.reverseSpeed;
              }
              if (floorForwardHeld) {
                return FloorConstants.forwardSpeed;
              }
              return 0.0;
            }));

    controller.povLeft().whileTrue(intake.intakeCommand());
    controller.povRight().whileTrue(intake.outtakeCommand());
    controller.povDown().whileTrue(intake.pivotDownCommand());
    controller.povUp().whileTrue(intake.pivotUpCommand());

    // Reset gyro to 0° when B button is pressed

    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
