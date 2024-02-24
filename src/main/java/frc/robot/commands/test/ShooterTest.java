// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterTest extends Command {
	
	protected final Shooter shooter;
	
	protected final BooleanSupplier shouldRunShooter;
	
	public ShooterTest(Shooter shooter, BooleanSupplier shouldRunShooter) {
		
		this.shooter = shooter;
		this.shouldRunShooter = shouldRunShooter;
		
		addRequirements(shooter);
		
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
		this.shooter.stop();
		
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		
		if (this.shouldRunShooter.getAsBoolean()) this.shooter.shoot();
		else this.shooter.stop();
		
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		
		this.shooter.stop();
		
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		
		return false;
		
	}
	
}
