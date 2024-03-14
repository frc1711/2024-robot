// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.DIODevice;
import frc.robot.controlsschemes.ControlsScheme;
import frc.robot.controlsschemes.SingleControllerTeleoperativeControlsScheme;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Seconds;

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
	
	public final Triggers triggers;
	
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
		this.controlsScheme = new SingleControllerTeleoperativeControlsScheme();
		this.commands = new RobotContainer.Commands();
		this.triggers = new RobotContainer.Triggers();
		
		Shuffleboard.getTab("Subsystems").add("Swerve", this.swerve);
		Shuffleboard.getTab("Subsystems").add("Shooter", this.shooter);
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
		
		protected Command prepareToShootAtAngle(
			Measure<Angle> angle,
			double shooterSpeed
		) {
			
			Arm.Commands arm = RobotContainer.this.arm.commands;
			Shooter.Commands shooter = RobotContainer.this.shooter.commands;
			
			return arm.goToRestingAngle(angle)
				.alongWith(shooter.spinUp(shooterSpeed, 0.05));
			
		}
		
		/**
		 * Runs the intake for a short period of time to feed a NOTE into the
		 * shooter.
		 *
		 * @return A command that runs the intake for a short period of time to
		 * feed a NOTE into the shooter.
		 */
		protected Command feedShooter() {
			
			Intake.Commands intake = RobotContainer.this.intake.commands;
			DigitalInput upperBeamBreakSensor =
				RobotContainer.this.upperBeamBreakSensor;
			
			// Run the intake until the upper beam break sensor is tripped.
			return intake.intake()
				.until(upperBeamBreakSensor::get)
				// ...and then run the intake for another 1/4 of a second.
				.andThen(intake.intake())
				.withTimeout(0.25)
				// ...and then run the intake until the upper beam break sensor
				// is NO LONGER tripped.
				.andThen(intake.intake())
				.until(() -> !upperBeamBreakSensor.get());
			
		}
		
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
			
			return this.shootAtAngle(
				angle,
				shooterSpeed,
				1.5,
				0.5
			);
			
		}
		
		public Command shootAtAngle(
			Measure<Angle> angle,
			double shooterSpeed,
			double armMovementTimeout,
			double armSettlingTime
		) {
			
			return this.prepareToShootAtAngle(angle, shooterSpeed)
				.withTimeout(armMovementTimeout)
				.andThen(new WaitCommand(armSettlingTime))
				.andThen(this.feedShooter())
				.finallyDo(() -> {
					RobotContainer.this.shooter.stop();
					RobotContainer.this.arm.setRestingAngle(Degrees.of(0));
				});
			
		}
		
		public Command shootBelliedUpToSubwoofer() {
			
			return this.shootAtAngle(
				Degrees.of(55),
				1,
				1,
				0.25
			);
			
		}
		
		public Command makeToast() {
			
			return this.prepareToShootAtAngle(Degrees.of(95), 0.145)
				.andThen(this.feedShooter())
				.andThen(new WaitCommand(1))
				.finallyDo(() -> {
					RobotContainer.this.shooter.stop();
					RobotContainer.this.arm.setRestingAngle(Degrees.of(0));
				});
			
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
		
		public Command grabNoteAndReturn(
			Measure<Angle> noteHeading,
			double speed,
			Measure<Time> driveTime
		) {
			
			Swerve.Commands swerve = RobotContainer.this.swerve.commands;
			Measure<Time> driveTrainSettlingWaitTime = Seconds.of(0.25);
			
			Command driveToNote = swerve.driveForTime(
				noteHeading,
				speed,
				Degrees.of(0),
				driveTime
			);
			
			Command waitForDriveTrainToSettle = new WaitCommand(
				driveTrainSettlingWaitTime.in(Seconds)
			);
			
			Command returnToSubwoofer = swerve.driveForTime(
				noteHeading.plus(Rotations.of(0.5)),
				speed,
				Degrees.of(0),
				driveTime.plus(Seconds.of(0.05))
			);
			
			Command spinUpShooter = new InstantCommand(
				RobotContainer.this.shooter::shoot
			);
			
			Command intakeNote = this.intakeUntilNoteIsReady()
				.withTimeout(4);
			
			return driveToNote
				.andThen(waitForDriveTrainToSettle)
				.andThen(returnToSubwoofer/*.alongWith(spinUpShooter)*/)
				.alongWith(intakeNote);
			
		}
		
		public Command grabNote1FromMiddlePosition(
			double speed,
			Measure<Time> driveTime
		) {
			
			return this.grabNoteAndReturn(
				Degrees.of(52),
				speed,
				driveTime
			);
			
		}
		
		public Command grabNote2FromMiddlePosition(
			double speed,
			Measure<Time> driveTime
		) {
			
			return this.grabNoteAndReturn(
				Degrees.of(0),
				speed,
				driveTime
			);
			
		}
		
		public Command grabNote3FromMiddlePosition(
			double speed,
			Measure<Time> driveTime
		) {
			
			return this.grabNoteAndReturn(
				Degrees.of(-50),
				speed,
				driveTime
			);
			
		}
		
	}
	
	public class Triggers {
		
		public Trigger isRobotPreparedToShoot(
			Measure<Angle> angle,
			double shooterSpeed
		) {
			
			Arm.Triggers arm = RobotContainer.this.arm.triggers;
			Shooter.Triggers shooter = RobotContainer.this.shooter.triggers;
			
			return arm.armIsAtAngle(angle, Degrees.of(1));
//				.and(shooter.isAtSpeed(shooterSpeed, 0.05));
			
		}
		
	}
	
}
