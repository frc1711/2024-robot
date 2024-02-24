// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmCommand extends Command {
	
	protected static final double SPEED = 0.3;
	
	protected final Arm arm;
	
	protected final BooleanSupplier shouldRaiseArm, shouldLowerArm;
	
	public ArmCommand(Arm arm, BooleanSupplier shouldRaiseArm, BooleanSupplier shouldLowerArm) {
		
		this.arm = arm;
		this.shouldRaiseArm = shouldRaiseArm;
		this.shouldLowerArm = shouldLowerArm;
		
		addRequirements(arm);
		
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
		this.arm.stop();
		
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		
		if (this.shouldRaiseArm.getAsBoolean()) this.arm.rotate(true);
		else if (this.shouldLowerArm.getAsBoolean()) this.arm.rotate(false);
		else this.arm.stop();
		
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		
		this.arm.stop();
		
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		
		return false;
		
	}
	
}
