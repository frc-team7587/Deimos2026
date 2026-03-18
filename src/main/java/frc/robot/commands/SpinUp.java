// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Fuel;
import static frc.robot.Constants.FuelConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinUp extends Command {
  /** Creates a new Intake. */

  Fuel fuelSubsystem;

  public SpinUp(Fuel fuelSystem) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for intaking
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {


    fuelSubsystem.setFeederRoller(LAUNCH_MOTOR_SPEED * RobotContainer.operatorController.getRightTriggerAxis());
    fuelSubsystem.setIntakeLauncherRoller(INTAKING_INTAKE_ROLLER_SPEED * RobotContainer.operatorController.getRightTriggerAxis());

    /* 
    double feederVelocity = fuelSubsystem.feederRoller.getEncoder().getVelocity();
    double feederTargetSpeed = fuelSubsystem.feederPIDController.calculate(feederVelocity, shooterRPM);

    fuelSubsystem.setFeederRoller(feederTargetSpeed);
    */
  
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
    fuelSubsystem.setFeederRoller(0);
    fuelSubsystem.setIntakeLauncherRoller(0);}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
