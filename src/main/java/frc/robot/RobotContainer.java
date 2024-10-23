// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.configuration.DIODevice;
import frc.robot.configuration.DoublePreference;
import frc.robot.controlsschemes.ControlsScheme;
import frc.robot.controlsschemes.SingleControllerTeleoperativeControlsScheme;
import frc.robot.controlsschemes.StandardTeleoperativeControlsScheme;
import frc.robot.devicewrappers.RaptorsLaserCanBooleanSupplier;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.urcl.URCL;

import java.util.function.BooleanSupplier;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {
	
	public final Swerve swerve;
	
	public final Shooter shooter;
	
	public final Intake intake;
	
	public final Arm arm;
	
	public final RaptorsLaserCanBooleanSupplier upperBeamBreakSensor;
	
	protected final CommandXboxController driveController;
	
	protected final CommandXboxController subsystemController;
	
//	protected final ControlsScheme controlsScheme;
	
	public final Commands commands;
	
	public RobotContainer() {
		
		// Initialize the subsystems.
		this.swerve = new Swerve();
		this.shooter = new Shooter();
		this.intake = new Intake();
		this.arm = new Arm();
		
		this.upperBeamBreakSensor = new RaptorsLaserCanBooleanSupplier(
			DIODevice.INTAKE_UPPER_BEAM_BREAK_SENSOR.id,
			(distance) -> distance.in(Inches) < 8
		);
		
		// Initialize the controller instances.
		this.driveController = new CommandXboxController(0);
		this.subsystemController = new CommandXboxController(1);
		
		// Initialize the controls scheme.
//		this.controlsScheme = new SingleControllerTeleoperativeControlsScheme();
		this.commands = new RobotContainer.Commands();
		
		Shuffleboard.getTab("Subsystems").add("Swerve", this.swerve);
		Shuffleboard.getTab("Subsystems").add("Shooter", this.shooter);
		// Shuffleboard.getTab("Subsystems").add("Intake", this.intake);
		Shuffleboard.getTab("Subsystems").add("Arm", this.arm);
		Shuffleboard.getTab("Subsystems").addBoolean(
			"Upper Beambreak",
			this.upperBeamBreakSensor::getAsBoolean
		);
		
		this.driveController.a().whileTrue(this.swerve.commands.quasistatic(SysIdRoutine.Direction.kForward));
		this.driveController.x().whileTrue(this.swerve.commands.quasistatic(SysIdRoutine.Direction.kReverse));
		this.driveController.y().whileTrue(this.swerve.commands.dynamic(SysIdRoutine.Direction.kForward));
		this.driveController.b().whileTrue(this.swerve.commands.dynamic(SysIdRoutine.Direction.kReverse));
		
	}
	
	public void robotInit() {
		
		DataLogManager.start();
		URCL.start();
		this.swerve.calibrateFieldRelativeHeading();
		
	}
	
	public void robotPeriodic() {
	
	
	
	}
	
	public void initTeleop() {
		
//		this.controlsScheme.configureControls(
//			this,
//			this.driveController,
//			this.subsystemController
//		);
		
	}
	
	public void teleopPeriodic() {
		
//		this.controlsScheme.periodic(
//			this,
//			this.driveController,
//			this.subsystemController
//		);
		
	}
	
	public void teleopExit() {
		
//		this.controlsScheme.exit(
//			this,
//			this.driveController,
//			this.subsystemController
//		);
		
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
			Command intakeUntilBeamBreak = intake.intake()
				.until(RobotContainer.this.upperBeamBreakSensor);
			Command intakeUntilPastBeamBreak = intake.intake().until(
				() -> !RobotContainer.this.upperBeamBreakSensor.getAsBoolean()
			);
			
			return intakeUntilBeamBreak
				.andThen(intakeUntilPastBeamBreak)
				.withTimeout(1);
		
		}
		
		public Command correctNotePosition() {
			
			Intake.Commands intake = RobotContainer.this.intake.commands;
			Shooter.Commands shooter = RobotContainer.this.shooter.commands;
			BooleanSupplier upperBeamBreakIsNotBroken =
				() -> !RobotContainer.this.upperBeamBreakSensor.getAsBoolean();
			
			Command quickCorrect = intake.outtake(0.5)
				.alongWith(shooter.shoot(-0.5))
				.until(upperBeamBreakIsNotBroken);
			
			Command advanceToSensor = intake.intake(0.25)
				.until(RobotContainer.this.upperBeamBreakSensor);
			
			Command accurateCorrect = intake.outtake(0.1)
				.until(upperBeamBreakIsNotBroken);
			
			Command correctNotePosition = quickCorrect
				.andThen(advanceToSensor)
				.andThen(accurateCorrect);
			
			return correctNotePosition.unless(upperBeamBreakIsNotBroken);
			
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
				0.5
			);
			
		}
		
		public Command makeToast() {
			
			Arm.Commands arm = RobotContainer.this.arm.commands;
			Swerve.Commands swerve = RobotContainer.this.swerve.commands;
			
			Command makeToast =
				this.prepareToShootAtAngle(Degrees.of(92), 0.13)
					.andThen(this.feedShooter())
					.andThen(new WaitCommand(0.05))
					.andThen(arm.holdAtAngle(Degrees.of(82)).withTimeout(0.2))
					.andThen(arm.holdAtAngle(Degrees.of(91)).withTimeout(1))
					.finallyDo(() -> {
						RobotContainer.this.shooter.stop();
						RobotContainer.this.arm.setRestingAngle(Degrees.of(0));
					});
			
			return swerve.slowDownWhile(0.3, makeToast);
			
		}
		
		/**
		 * Runs the intake until the upper beam break sensor is triggered.
		 *
		 * @return A command that runs the intake until the upper beam break
		 * sensor is triggered.
		 */
		public Command intakeUntilNoteIsReady() {
			
			return RobotContainer.this.intake.commands.featherIntake()
				.until(RobotContainer.this.upperBeamBreakSensor)
				.andThen(new InstantCommand(() -> RobotContainer.this.swerve.setSpeedMultiplier(1)))
				.andThen(this.correctNotePosition());
			
		}
		
		public Command grabNoteAndReturn(
			Measure<Angle> noteHeading,
			double speed,
			Measure<Time> driveTime
		) {
			
			Swerve.Commands swerve = RobotContainer.this.swerve.commands;
			Measure<Time> driveTrainSettlingWaitTime = Seconds.of(0.25);
			
			Command driveToNote = swerve.driveForTime2(
				noteHeading,
				speed,
				Degrees.of(0),
				driveTime
			);
			
			Command waitForDriveTrainToSettle = new WaitCommand(
				driveTrainSettlingWaitTime.in(Seconds)
			);
			
			Command returnToSubwoofer = swerve.driveForTime2(
				noteHeading.plus(Rotations.of(0.5)),
				speed,
				Degrees.of(0),
				driveTime
			);
			
			Command intakeNote = this.intakeUntilNoteIsReady()
				.withTimeout(4);
			
			return driveToNote
				.andThen(waitForDriveTrainToSettle)
				.andThen(returnToSubwoofer)
				.alongWith(intakeNote);
			
		}
		
		public Command grabNote1FromMiddlePosition() {
			
			return this.grabNoteAndReturn(
				Degrees.of(52),
				1,
				Seconds.of(1.2)
			);
			
		}
		
		public Command grabNote2FromMiddlePosition() {
			
			return this.grabNoteAndReturn(
				Degrees.of(0),
				1,
				Seconds.of(0.9)
			);
			
		}
		
		public Command grabNote3FromMiddlePosition() {
			
			return this.grabNoteAndReturn(
				Degrees.of(-50),
				1,
				Seconds.of(1.2)
			);
			
		}
		
		public Command waitToStartAuton() {
			
			return new WaitCommand(DoublePreference.AUTON_START_DELAY.get());
			
		}
		
		public Command waitToRolloutInAuton() {
			
			return new WaitCommand(DoublePreference.AUTON_ROLLOUT_DELAY.get());
			
		}
		
	}
	
}
