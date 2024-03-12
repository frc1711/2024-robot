// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework.basic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;

import static edu.wpi.first.units.Units.Degrees;

public class SwerveAuton extends Command {
	
	Swerve swerveSubsystem;
	
	double xSpeed, ySpeed;
	
	Rotation2d desiredRotation;
	
	PIDController rotationalPID;
	
	public SwerveAuton(RobotContainer robot, double xSpeed, double ySpeed, Rotation2d desiredRotation) {
		
		this.swerveSubsystem = robot.swerve;
		this.xSpeed = xSpeed;
		this.ySpeed = ySpeed;
		this.desiredRotation = desiredRotation;
		
		addRequirements(swerveSubsystem);
		
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
		swerveSubsystem.stop();
		swerveSubsystem.setFieldRelativeHeadingSetpoint(Degrees.of(this.desiredRotation.getDegrees()));

	}
	
	@Override
	public void execute() {
		
		this.swerveSubsystem.applyChassisSpeeds(
			new ChassisSpeeds(xSpeed, ySpeed, 0),
			true
		);
		
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		
		swerveSubsystem.stop();
		
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		
		return false;
		
	}
	
}
