// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework.basic;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;

public class IntakeAuton extends ParallelDeadlineGroup {
	
	public IntakeAuton(Intake intakeSubsystem) {
		
		super (new WaitCommand(1.5));
		
		addCommands(intakeSubsystem.commands.intake());
	}
	
}
