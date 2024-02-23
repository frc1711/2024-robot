// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework.basic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterAuton extends Command {
	
	Shooter shooterSubsystem;
	
	Timer timer;
	
	public ShooterAuton(Shooter shooterSubsystem) {
		
		this.shooterSubsystem = shooterSubsystem;
		this.timer = new Timer();
		addRequirements(shooterSubsystem);
		
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
		shooterSubsystem.stop();
		timer.restart();
		
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		
		shooterSubsystem.runShooter(1, 1);
		
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		
		shooterSubsystem.stop();
		timer.stop();
		
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		
		return timer.hasElapsed(2);
		
	}
	
}
