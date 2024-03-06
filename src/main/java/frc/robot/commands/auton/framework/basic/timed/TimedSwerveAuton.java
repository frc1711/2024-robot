// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework.basic.timed;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class TimedSwerveAuton extends Command {
	
	Swerve swerveSubsystem;

	ChassisSpeeds speed;

	double time;
	
	Timer timer;
	
	public TimedSwerveAuton(Swerve swerveSubsystem, ChassisSpeeds speed, double time) {
		
		this.swerveSubsystem = swerveSubsystem;
		this.speed = speed;
		this.time = time;
		timer = new Timer();
		timer.start();
		
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
		swerveSubsystem.stop();
		
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		
		swerveSubsystem.applyChassisSpeeds(
			speed
		);
		
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		
		swerveSubsystem.stop();
		timer.stop();
		
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		
		return timer.hasElapsed(time);
		
	}
	
}
