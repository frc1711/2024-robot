// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auton.framework.basic.IntakeAuton;
import frc.robot.commands.auton.framework.basic.SwerveAuton;
import frc.robot.commands.auton.framework.basic.timed.TimeBasedSwerveAuton;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupAuton extends ParallelDeadlineGroup {
	
	double xVariable, yVariable;

	Pose2d robotPose;

	/**
	 * Creates a new PickupAuton.
	 */
	public PickupAuton(RobotContainer robot) {
		
		super(new IntakeAuton(robot));

		robot.swerveSubsystem.calibrateFieldRelativeHeading();

		xVariable = Math.cos(robot.swerveSubsystem.getFieldRelativeHeadingRotation2d().getRadians());
		yVariable = Math.sin(robot.swerveSubsystem.getFieldRelativeHeadingRotation2d().getRadians());
		robotPose = robot.swerveSubsystem.getRobotPose();
		
		addCommands(new SwerveAuton(robot, .15, 0, robot.swerveSubsystem.getFieldRelativeHeadingRotation2d()).raceWith(new WaitCommand(3)));
		
	}
	
}