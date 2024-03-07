// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.framework.PickupAuton;
import frc.robot.commands.auton.framework.SpeakerAuton;
import frc.robot.commands.auton.framework.basic.ArmAuton;
import frc.robot.commands.auton.framework.basic.BellyUpSpeaker;
import frc.robot.commands.auton.framework.basic.OdometryAuton;
import frc.robot.commands.auton.framework.basic.SwerveAuton;
import frc.robot.commands.auton.framework.basic.timed.TimeBasedSwerveAuton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class King extends SequentialCommandGroup {
  
  public King(Swerve swerveSubsystem, Intake intakeSubsystem, Shooter shooterSubsystem, Arm armSubsystem) {
    //TODO: Decide which notes to pick up in autons to determine angle to the speaker's AprilTag and their positions relative to the robot
    addCommands(
      armSubsystem.commands.goToAngle(Degrees.of(55)),
      new BellyUpSpeaker(armSubsystem, shooterSubsystem, intakeSubsystem),
      new PickupAuton(intakeSubsystem, swerveSubsystem),
      new TimeBasedSwerveAuton(new SwerveAuton(swerveSubsystem, -.25, 0, new Rotation2d(Degrees.of(0))), 1.5),
      new BellyUpSpeaker(armSubsystem, shooterSubsystem, intakeSubsystem),
      new TimeBasedSwerveAuton(new SwerveAuton(swerveSubsystem, 0, 0, new Rotation2d()), 0),
      new PickupAuton(intakeSubsystem, swerveSubsystem),
      new TimeBasedSwerveAuton(new SwerveAuton(swerveSubsystem, 0, 0, new Rotation2d()), 0),
      new BellyUpSpeaker(armSubsystem, shooterSubsystem, intakeSubsystem)
      );
  }
}
