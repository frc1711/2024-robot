// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework.basic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BellyUpSpeaker extends Command {

  Shooter shooter;
  Arm arm;
  Intake intake;
  Timer timer;

  public BellyUpSpeaker(Arm arm, Shooter shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;
    this.arm = arm;
    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stop();
    intake.stop();
    arm.stop();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.rotateToAngle(55);
    shooter.shoot();
    if (timer.hasElapsed(3)) intake.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed((4));
  }
}
