// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework.basic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class RotateAuton extends Command {
	
	Swerve swerveSubsystem;
	
	Rotation2d rotation;
	
	PIDController rotationalPID;
	
	public RotateAuton(Swerve swerveSubsystem, Rotation2d rotation) {
		
		this.swerveSubsystem = swerveSubsystem;
		this.rotation = rotation;
		
		rotationalPID = new PIDController(1, 0, 0);
		
		rotationalPID.enableContinuousInput(0, 360);
		rotationalPID.setTolerance(1);
		
		addRequirements(swerveSubsystem);
		
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
		swerveSubsystem.stop();
		
	}
	
	double rotationSpeed;
	
	@Override
	public void execute() {
		
		rotationSpeed = rotationalPID.calculate(
			swerveSubsystem.getFieldRelativeHeadingRotation2d().getDegrees(),
			rotation.getDegrees()
		);
		
		swerveSubsystem.applyChassisSpeeds(
			new ChassisSpeeds(0, 0, rotationSpeed),
			false
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
		
		return rotationalPID.atSetpoint();
		
	}
	
}
