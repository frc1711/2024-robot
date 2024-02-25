// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework.basic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterAuton extends Command {
	
	protected final Shooter shooter;
	
	protected final Timer timer;
	
	public ShooterAuton(Shooter shooter) {
		
		this.shooter = shooter;
		this.timer = new Timer();
		
		addRequirements(shooter);
		
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
		this.shooter.stop();
		this.timer.restart();
		
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		
		this.shooter.shoot();
		
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		
		this.shooter.stop();
		this.timer.stop();
		
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		
		return this.timer.hasElapsed(3);
		
	}
	
}
