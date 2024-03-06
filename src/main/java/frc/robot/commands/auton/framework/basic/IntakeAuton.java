// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework.basic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeAuton extends Command {
	
	Intake intakeSubsystem;

	Timer timer;
	
	public IntakeAuton(Intake intakeSubsystem) {
		
		this.intakeSubsystem = intakeSubsystem;
		addRequirements(intakeSubsystem);
		
		timer = new Timer();
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
		intakeSubsystem.stop();

		timer.start();
		
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		
		intakeSubsystem.intake();
		
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		
		intakeSubsystem.stop();
		
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		
		return intakeSubsystem.isHoldingNote() || timer.hasElapsed(3);
		
	}
	
}
