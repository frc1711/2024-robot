// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
	public PickupAuton(Intake intake, Swerve swerve) {
		
		super(new IntakeAuton(intake));

		xVariable = Math.cos(swerve.getFieldRelativeHeadingRotation2d().getRadians());
		yVariable = Math.sin(swerve.getFieldRelativeHeadingRotation2d().getRadians());
		robotPose = swerve.getRobotPose();
		
		addCommands(new TimeBasedSwerveAuton(new SwerveAuton(swerve, yVariable, xVariable, null), 2));
		
	}
	
}