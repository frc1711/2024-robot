// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.configuration.DIODevice;
import frc.robot.controlsschemes.ControlsScheme;
import frc.robot.controlsschemes.SingleControllerTeleoperativeControlsScheme;
import frc.robot.controlsschemes.StandardTeleoperativeControlsScheme;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
	
	public final Swerve swerve;
	
	public final Shooter shooter;
	
	public final Intake intake;
	
	public final Arm arm;
	
	public final DigitalInput upperBeamBreakSensor;
	
	public final DigitalInput lowerBeamBreakSensor;
	
	protected final CommandXboxController driveController;
	
	protected final CommandXboxController subsystemController;
	
	protected final ControlsScheme controlsScheme;
	
	public final Commands commands;
	
	public RobotContainer() {
		
		// Initialize the subsystems.
		this.swerve = new Swerve();
		this.shooter = new Shooter();
		this.intake = new Intake();
		this.arm = new Arm();
		
		// Initialize the various sensors on the robot.
		this.upperBeamBreakSensor =
			new DigitalInput(DIODevice.INTAKE_UPPER_BEAM_BREAK_SENSOR.id);
		
		this.lowerBeamBreakSensor =
			new DigitalInput(DIODevice.INTAKE_LOWER_BEAM_BREAK_SENSOR.id);
		
		// Initialize the controller instances.
		this.driveController = new CommandXboxController(0);
		this.subsystemController = new CommandXboxController(1);
		
		// Initialize the controls scheme.
		this.controlsScheme = new StandardTeleoperativeControlsScheme();
		this.commands = new RobotContainer.Commands();
		
		Shuffleboard.getTab("Subsystems").add("Swerve", this.swerve);
		// Shuffleboard.getTab("Subsystems").add("Shooter", this.shooter);
		// Shuffleboard.getTab("Subsystems").add("Intake", this.intake);
		Shuffleboard.getTab("Subsystems").add("Arm", this.arm);
		
	}
	
	public void initTeleop() {
		
		this.controlsScheme.configureControls(
			this,
			this.driveController,
			this.subsystemController
		);
		
	}
	
	public void teleopPeriodic() {
		
		this.controlsScheme.periodic(
			this,
			this.driveController,
			this.subsystemController
		);
		
	}
	
	public void teleopExit() {
		
		this.controlsScheme.exit(
			this,
			this.driveController,
			this.subsystemController
		);
		
	}
	
	public class Commands {
		
		/**
		 * Shoots a NOTE at the specified angle and speed (assuming that the
		 * robot already has a NOTE in place, ready to shoot).
		 *
		 * @param angle The angle at which to shoot the NOTE.
		 * @param shooterSpeed The speed at which to shoot the NOTE.
		 * @return A command that shoots a NOTE at the specified angle and
		 * speed.
		 */
		public Command shootAtAngle(
			Measure<Angle> angle,
			double shooterSpeed
		) {
			
			Arm.Commands arm = RobotContainer.this.arm.commands;
			Shooter.Commands shooter = RobotContainer.this.shooter.commands;
			Intake.Commands intake = RobotContainer.this.intake.commands;
			
			Command prepareToShoot = arm.holdAtAngle(angle)
				.alongWith(shooter.spinUp(shooterSpeed));
			
			Command waitUntilPreparedToShoot = new WaitCommand(1.5)
				.until(RobotContainer.this.arm.getController()::atSetpoint);
			
			Command shoot = intake.intake().withTimeout(1);
			
			return waitUntilPreparedToShoot
				.andThen(shoot)
				.deadlineWith(prepareToShoot)
				.finallyDo(RobotContainer.this.shooter::stop);
			
		}
		
		/**
		 * Runs the intake until the upper beam break sensor is triggered.
		 *
		 * @return A command that runs the intake until the upper beam break
		 * sensor is triggered.
		 */
		public Command intakeUntilNoteIsReady() {
			
			Intake.Commands intake = RobotContainer.this.intake.commands;
			DigitalInput upperBeamBreakSensor =
				RobotContainer.this.upperBeamBreakSensor;
			
			return intake.intake().until(upperBeamBreakSensor::get);
			
		}
		
	}
	
}
