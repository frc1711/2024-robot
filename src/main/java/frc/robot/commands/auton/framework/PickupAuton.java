// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.auton.framework.basic.IntakeAuton;
import frc.robot.commands.auton.framework.basic.OdometryAuton;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupAuton extends ParallelDeadlineGroup {
	
	/**
	 * Creates a new PickupAuton.
	 */
	public PickupAuton(Intake intake, Swerve swerve) {
		
		super(new IntakeAuton(intake));
		
		addCommands(new OdometryAuton(swerve, swerve.getRobotPose(), 1));
		
	}
	
}
