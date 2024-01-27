// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework.basic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmAuton extends Command {
  
  Arm armSubsystem;
  double angleInDegrees;

  double acceptableError = .1;

  public ArmAuton(Arm armSubsystem, double angleInDegrees) {
    this.armSubsystem = armSubsystem;
    this.angleInDegrees = angleInDegrees;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setToAngle(angleInDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((angleInDegrees - acceptableError) <= armSubsystem.getAngleDegrees() && armSubsystem.getAngleDegrees() <= (angleInDegrees + acceptableError))
    return true;
    else return false;
  }
}
