// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework.basic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class SwerveAuton extends Command {
	
	Swerve swerveSubsystem;
	
	double xSpeed, ySpeed;
	
	Rotation2d desiredRotation;
	
	PIDController rotationalPID;
	
	public SwerveAuton(Swerve swerveSubsystem, double xSpeed, double ySpeed, Rotation2d desiredRotation) {
		
		this.swerveSubsystem = swerveSubsystem;
		this.xSpeed = xSpeed;
		this.ySpeed = ySpeed;
		this.desiredRotation = desiredRotation;
		this.rotationalPID = new PIDController(.01, 0, 0);
		rotationalPID.enableContinuousInput(0, 360);
		addRequirements(swerveSubsystem);
		
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
		swerveSubsystem.stop();
		
		swerveSubsystem.configureChassisSpeedInputs(() -> xSpeed,() -> ySpeed, () -> thetaSpeed);

	}
	
	double thetaSpeed;
	
	@Override
	public void execute() {
		
		thetaSpeed = rotationalPID.calculate(
			swerveSubsystem.getFieldRelativeHeadingRotation2d().getDegrees(),
			desiredRotation
		.getDegrees());
		
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		
		swerveSubsystem.configureChassisSpeedInputs(() -> 0, () -> 0, () -> 0);
		swerveSubsystem.stop();
		
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		
		return false;
		
	}
	
}
