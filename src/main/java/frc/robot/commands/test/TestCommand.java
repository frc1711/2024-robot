// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TestCommand extends Command {
  
  Subsystem subsystem;
  InstantCommand runSubsystem, stopSubsystem, increaseSpeedCommand, decreaseSpeedCommand;
  BooleanSupplier increaseSpeed, decreaseSpeed, eStop, tempStop;

  public TestCommand(Subsystem subsystem, InstantCommand runSubsystem, InstantCommand stopSubsystem, InstantCommand increaseSpeedCommand, InstantCommand decreaseSpeedCommand, BooleanSupplier increaseSpeed, BooleanSupplier decreaseSpeed, BooleanSupplier eStop, BooleanSupplier tempStop) {
    this.subsystem = subsystem;
    this.runSubsystem = runSubsystem;
    this.stopSubsystem = stopSubsystem;
    this.increaseSpeed = increaseSpeed;
    this.decreaseSpeed = decreaseSpeed;
    this.increaseSpeedCommand = increaseSpeedCommand;
    this.decreaseSpeedCommand = decreaseSpeedCommand;
    this.eStop = eStop;
    this.tempStop = tempStop;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopSubsystem.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (increaseSpeed.getAsBoolean()) increaseSpeedCommand.schedule();

    else if (decreaseSpeed.getAsBoolean()) decreaseSpeedCommand.schedule();

    if (tempStop.getAsBoolean()) stopSubsystem.schedule();
    
    else runSubsystem.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stopSubsystem.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return eStop.getAsBoolean();
  }
}
