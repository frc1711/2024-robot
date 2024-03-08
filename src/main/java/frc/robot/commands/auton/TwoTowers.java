// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ComplexCommands;
import frc.robot.RobotContainer;
import frc.robot.commands.auton.framework.PickupAuton;
import frc.robot.commands.auton.framework.basic.SwerveAuton;
import frc.robot.commands.auton.framework.basic.timed.TimeBasedSwerveAuton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoTowers extends SequentialCommandGroup {
  
  public TwoTowers(RobotContainer robotContainer, Swerve swerveSubsystem, Intake intakeSubsystem, Shooter shooterSubsystem, Arm armSubsystem) {
    
    //TODO: Determine timing and directions for drive autons
    addCommands(
      ComplexCommands.prepareToShootAtAngle(armSubsystem, shooterSubsystem, Degrees.of(55), 1),
      ComplexCommands.finishShootingAtAngle(armSubsystem, shooterSubsystem, intakeSubsystem, Degrees.of(55), 1).raceWith(new WaitCommand(2)),
      new SwerveAuton(swerveSubsystem, 0, 0, new Rotation2d(Degrees.of(-30))).raceWith(new WaitCommand(0)),
      new PickupAuton(robotContainer, intakeSubsystem, swerveSubsystem),
      new SwerveAuton(swerveSubsystem, -.15, 0, swerveSubsystem.getFieldRelativeHeadingRotation2d()).raceWith(new WaitCommand(3)),
      ComplexCommands.prepareToShootAtAngle(armSubsystem, shooterSubsystem, Degrees.of(55), 1),
      ComplexCommands.finishShootingAtAngle(armSubsystem, shooterSubsystem, intakeSubsystem, Degrees.of(55), 1).raceWith(new WaitCommand(2)));
      // new SwerveAuton(swerveSubsystem, 0, 0, new Rotation2d()).raceWith(new WaitCommand(0)),
      // new PickupAuton(intakeSubsystem, swerveSubsystem),
      // new TimeBasedSwerveAuton(new SwerveAuton(swerveSubsystem, 0, 0, new Rotation2d()), 0),
      // ComplexCommands.prepareToShootAtAngle(armSubsystem, shooterSubsystem, Degrees.of(55), 1),
      // ComplexCommands.finishShootingAtAngle(armSubsystem, shooterSubsystem, intakeSubsystem, Degrees.of(55), 1));
  }
}
