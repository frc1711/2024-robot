// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auton.framework.basic.OdometryAuton;
import frc.robot.commands.auton.framework.basic.ShooterAuton;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicAuton extends ParallelCommandGroup {
  
  public BasicAuton(Swerve swerve, Shooter shooter) {
    
    addCommands(new OdometryAuton(swerve, new Pose2d(swerve.getRobotPose().getX() + 1, swerve.getRobotPose().getY(), swerve.getGyroRotation()), .5), new ShooterAuton(shooter));
  }
}
