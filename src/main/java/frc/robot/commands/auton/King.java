// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.auton.framework.PickupAuton;
import frc.robot.commands.auton.framework.basic.SwerveAuton;
import frc.robot.commands.auton.framework.basic.timed.TimeBasedSwerveAuton;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class King extends SequentialCommandGroup {
  
  public King(RobotContainer robot) {
    //TODO: Decide which notes to pick up in autons to determine angle to the speaker's AprilTag and their positions relative to the robot
    addCommands(
      robot.arm.commands.goToAngle(Degrees.of(55)),
      robot.commands.shootAtAngle(Degrees.of(55), 1),
      new PickupAuton(robot),
      new TimeBasedSwerveAuton(new SwerveAuton(robot, -.25, 0, new Rotation2d(Degrees.of(0))), 1.5),
      robot.commands.shootAtAngle(Degrees.of(55), 1),
      new TimeBasedSwerveAuton(new SwerveAuton(robot, 0, 0, new Rotation2d()), 0),
      new PickupAuton(robot),
      new TimeBasedSwerveAuton(new SwerveAuton(robot, 0, 0, new Rotation2d()), 0),
      robot.commands.shootAtAngle(Degrees.of(55), 1)
    );
  }
}
