// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework.basic.timed;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auton.framework.basic.SwerveAuton;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TimeBasedSwerveAuton extends ParallelRaceGroup {
  /** Creates a new TimeBasedSwerveAuton. */
  public TimeBasedSwerveAuton(SwerveAuton swerveAuton, double time) {

    // super(new WaitCommand(time));

    addCommands(
      new WaitCommand(time),
      swerveAuton);
  }
}
