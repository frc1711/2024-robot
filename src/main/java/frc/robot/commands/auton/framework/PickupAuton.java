// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.auton.framework.basic.IntakeAuton;
import frc.robot.commands.auton.framework.basic.SwerveAuton;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupAuton extends ParallelRaceGroup {
  /** Creates a new PickupAuton. */
  public PickupAuton(Intake intake, Swerve swerve) {
    
    swerve.resetGyro(); //TODO: figure out a way to go directly in the way the robot faces without screwing up kinematics 

    addCommands(new IntakeAuton(intake), new SwerveAuton(swerve, .1, 0, 0));
  }
}
