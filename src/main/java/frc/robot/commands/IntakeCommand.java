// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
  
  Intake intake;
  BooleanSupplier runIntake, reverseMode;

  public IntakeCommand(Intake intake, BooleanSupplier runIntake, BooleanSupplier reverseMode) {
    this.intake = intake;
    this.runIntake = runIntake;
    this.reverseMode = reverseMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (runIntake.getAsBoolean()) intake.runIntake(reverseMode.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
