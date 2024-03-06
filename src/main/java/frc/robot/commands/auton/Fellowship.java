// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.framework.basic.ArmAuton;
import frc.robot.commands.auton.framework.basic.BellyUpSpeaker;
import frc.robot.commands.auton.framework.basic.OdometryAuton;
import frc.robot.commands.auton.framework.basic.ShooterAuton;
import frc.robot.constants.DoublePreference;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

import static edu.wpi.first.units.Units.Degrees;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Fellowship extends SequentialCommandGroup {
  
  public Fellowship(Swerve swerve, Shooter shooter, Intake intake, Arm arm) {
    
    addCommands(
        arm.commands.holdAtAngle(Degrees.of(55)),
        new BellyUpSpeaker(arm, shooter, intake),
        new OdometryAuton(
			swerve,
			new Pose2d(
				swerve.getRobotPose().getX() + 1,
				swerve.getRobotPose().getY(),
				swerve.getFieldRelativeHeadingRotation2d()
			)
		),
		new ShooterAuton(shooter)
    );
    
  }
}
