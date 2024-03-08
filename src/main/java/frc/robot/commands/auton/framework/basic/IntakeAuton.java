// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework.basic;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeAuton extends ParallelRaceGroup {
	
	public IntakeAuton(RobotContainer robotContainer, Intake intakeSubsystem) {
		
		addCommands(new WaitCommand(5), intakeSubsystem.commands.intake().onlyWhile(() -> !robotContainer.upperBeamBreakSensor.get()));
	}
	
}
