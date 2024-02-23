// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.framework.PickupAuton;
import frc.robot.commands.auton.framework.SpeakerAuton;
import frc.robot.commands.auton.framework.basic.OdometryAuton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BassAuton extends SequentialCommandGroup {
  
  public BassAuton(Swerve swerveSubsystem, Intake intakeSubsystem, Shooter shooterSubsystem, Arm armSubsystem) {
    
    //TODO: Decide which notes to pick up in autons to determine angle to the speaker's AprilTag and their positions relative to the robot
    addCommands(
      new SpeakerAuton(swerveSubsystem, shooterSubsystem, armSubsystem, 0),
      new OdometryAuton(swerveSubsystem, new Pose2d(2.1, 6.8, new Rotation2d(0)), 1),
      new PickupAuton(intakeSubsystem, swerveSubsystem),
      new SpeakerAuton(swerveSubsystem, shooterSubsystem, armSubsystem, -153),
      new OdometryAuton(swerveSubsystem, new Pose2d(3.1, 6, new Rotation2d(-110)), 1),
      new PickupAuton(intakeSubsystem, swerveSubsystem),
      new SpeakerAuton(swerveSubsystem, shooterSubsystem, armSubsystem, 175));
  }
}
