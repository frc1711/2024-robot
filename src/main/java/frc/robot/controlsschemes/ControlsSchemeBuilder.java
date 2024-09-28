package frc.robot.controlsschemes;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.ControlsUtilities;
import frc.robot.util.DoubleSupplierBuilder;
import frc.robot.util.PointSupplierBuilder;

import java.util.Map;

import static edu.wpi.first.units.Units.Degrees;

/**
 * A builder for constructing controls schemes for the robot.
 */
public class ControlsSchemeBuilder {
	
	/**
	 * The threshold at which the triggers of the controller should be
	 * considered to be pressed.
	 */
	protected static final double TRIGGER_THRESHOLD = 0.5;
	
	/**
	 * The deadband to apply to the joysticks of the controller.
	 */
	protected static final double JOYSTICK_DEADBAND = 0.1;
	
	/**
	 * The power to raise the input of the joysticks to for smoothing.
	 */
	protected static final double LINEAR_INPUT_SMOOTHING_POWER = 2;
	
	/**
	 * The robot to build the controls scheme for.
	 */
	protected final RobotContainer robot;
	
	/**
	 * Initializes a new ControlsSchemeBuilder against the given robot.
	 *
	 * @param robot The robot to build the controls scheme for.
	 */
	public ControlsSchemeBuilder(RobotContainer robot) {
		
		this.robot = robot;
		
	}
	
	/**
	 * Configures the default commands for the robot's subsystems.
	 *
	 * @return This builder, for method chaining.
	 */
	public ControlsSchemeBuilder configureDefaultRobotCommands() {
		
		Arm.Commands arm = this.robot.arm.commands;
		Shooter.Commands shooter = this.robot.shooter.commands;
		
		this.robot.arm.setDefaultCommand(
			arm.holdAtAngle(Degrees.of(0))
		);
		
		this.robot.shooter.setDefaultCommand(shooter.stop());
		
		this.robot.intake.setDefaultCommand(
			this.robot.commands.correctNotePosition()
		);
		
		return this;
		
	}
	
	/**
	 * Registers the 'SELECT' button of the given controller to calibrate the
	 * field-relative heading of the robot.
	 *
	 * @param controller The controller that should be registered to handle the
	 * given controls.
	 * @return This builder, for method chaining.
	 */
	public ControlsSchemeBuilder useControllerSelectButtonToCalibrateFieldRelativeHeading(
		CommandXboxController controller
	) {
		
		controller.back().onTrue(
			this.robot.swerve.commands.calibrateFieldRelativeHeading()
		);
		
		return this;
		
	}
	
	/**
	 * Registers the left joystick of the given controller to control the
	 * strafing controls of the robot, and the right joystick to control the
	 * rotating controls of the robot.
	 *
	 * @param controller The controller that should be registered to handle the
	 * given controls.
	 * @return This builder, for method chaining.
	 */
	public ControlsSchemeBuilder useControllerJoysticksForDriving(
		CommandXboxController controller
	) {
		
		this.robot.swerve.setDefaultCommand(
			this.robot.swerve.commands.driveFieldRelative(
				PointSupplierBuilder.fromLeftJoystick(controller)
					.normalizeXboxJoystickToNWU()
					.withClamp(-1, 1)
					.withScaledDeadband(JOYSTICK_DEADBAND)
					.withExponentialCurve(LINEAR_INPUT_SMOOTHING_POWER),
				DoubleSupplierBuilder.fromRightX(controller)
					.withScaling(-1)
					.withClamp(-1, 1)
					.withScaledDeadband(JOYSTICK_DEADBAND)
					.withExponentialCurve(LINEAR_INPUT_SMOOTHING_POWER)
			)
		);
		
		return this;
		
	}
	
	/**
	 * Registers the cardinal directions of the D-Pad of the given controller to
	 * snap the robot to the corresponding field-relative heading.
	 *
	 * @param controller The controller that should be registered to handle the
	 * given controls.
	 * @return This builder, for method chaining.
	 */
	public ControlsSchemeBuilder useControllerDPadForSnappingToHeading(
		CommandXboxController controller
	) {
		
		Swerve.Commands swerve = this.robot.swerve.commands;
		
		controller.povUp().onTrue(swerve.setFieldRelativeHeading(Degrees.of(0)));
		controller.povLeft().onTrue(swerve.setFieldRelativeHeading(Degrees.of(90)));
		controller.povDown().onTrue(swerve.setFieldRelativeHeading(Degrees.of(180)));
		controller.povRight().onTrue(swerve.setFieldRelativeHeading(Degrees.of(270)));
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useBumpersForRelativeHeadingSnapping(
		CommandXboxController controller
	) {
		
		double increment = 90;
		
		controller.leftBumper().onTrue(this.robot.swerve.runOnce(() -> {
			
			double currentHeading = this.robot.swerve.getFieldRelativeHeading()
				.in(Degrees);
			
			currentHeading += increment;
			currentHeading = Math.rint(currentHeading/increment) * increment;
			currentHeading = ControlsUtilities.normalizeToRange(currentHeading, 0, 360);
			
			this.robot.swerve.setFieldRelativeHeadingSetpoint(Degrees.of(currentHeading));
			
		}));
		
		controller.rightBumper().onTrue(this.robot.swerve.runOnce(() -> {
			
			double currentHeading = this.robot.swerve.getFieldRelativeHeading()
				.in(Degrees);
			
			currentHeading -= increment;
			currentHeading = Math.rint(currentHeading/increment) * increment;
			currentHeading = ControlsUtilities.normalizeToRange(currentHeading, 0, 360);
			
			this.robot.swerve.setFieldRelativeHeadingSetpoint(Degrees.of(currentHeading));
			
		}));
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useRightJoystickClickToCorrectHeading(
		CommandXboxController controller
	) {
		
		double increment = 90;
		
		controller.rightStick().onTrue(this.robot.swerve.runOnce(() -> {
			
			double currentHeading = this.robot.swerve.getFieldRelativeHeading()
				.in(Degrees);
			
			currentHeading = Math.rint(currentHeading/increment) * increment;
			currentHeading = ControlsUtilities.normalizeToRange(currentHeading, 0, 360);
			
			this.robot.swerve.setFieldRelativeHeadingSetpoint(Degrees.of(currentHeading));
			
		}));
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useAButtonAsSlowMode(
		CommandXboxController controller
	) {
		
		controller.a().onTrue(new InstantCommand(() -> this.robot.swerve.setSpeedMultiplier(0.3)));
		controller.a().onFalse(new InstantCommand(() -> this.robot.swerve.setSpeedMultiplier(1)));
		
		return this;
		
	}
	
	/**
	 * Registers the triggers of the given controller to control the intake of
	 * the robot.
	 *
	 * @param controller The controller that should be registered to handle the
	 * given controls.
	 * @return This builder, for method chaining.
	 */
	public ControlsSchemeBuilder useControllerTriggersForIntakingAndOuttaking(
		CommandXboxController controller
	) {
		
		Intake.Commands intake = this.robot.intake.commands;
		Shooter.Commands shooter = this.robot.shooter.commands;
		
		// Spin the intake outwards while the left trigger is pressed.
		controller.leftTrigger(TRIGGER_THRESHOLD).whileTrue(
			intake.outtake().alongWith(shooter.shoot(-0.5))
		);
		
		// Spin the intake inwards while the right trigger is pressed.
		controller.rightTrigger(TRIGGER_THRESHOLD).whileTrue(
			new SelectCommand<>(Map.ofEntries(
				Map.entry(true, intake.intake()),
				Map.entry(false, this.robot.commands.intakeUntilNoteIsReady())
			), this.robot.upperBeamBreakSensor)
				.alongWith(new FunctionalCommand(
					() -> this.robot.swerve.setSpeedMultiplier(0.3),
					() -> {},
					(wasInterrupted) -> this.robot.swerve.setSpeedMultiplier(1),
					() -> false
				))
		);
		
		return this;
		
	}
	
	/**
	 * Periodically controls the rumble of the given controller based on the
	 * state of the intake's beam break sensor.
	 *
	 * @param controller The controller that should be registered to handle the
	 * given controls.
	 * @return This builder, for method chaining.
	 */
	public ControlsSchemeBuilder rumbleControllerWhenNoteInIntake(
		CommandXboxController controller
	) {
	
//		double rumbleIntensity =
//			this.robot.lowerBeamBreakSensor.get() ? 0.5 : 0;
//
//		controller.getHID().setRumble(
//			GenericHID.RumbleType.kBothRumble,
//			rumbleIntensity
//		);

		return this;
		
	}
	
	/**
	 * Registers the 'A', 'B', and 'Y' buttons of the given controller to shoot
	 * the robot's shooter at different angles.
	 *
	 * @param controller The controller that should be registered to handle the
	 * given controls.
	 * @return This builder, for method chaining.
	 */
	public ControlsSchemeBuilder useABXYButtonsForShooting(
		CommandXboxController controller
	) {
		
		RobotContainer.Commands robot = this.robot.commands;
		
		controller.x().whileTrue(robot.shootAtAngle(Degrees.of(55), 1).alongWith(new FunctionalCommand(
			() -> this.robot.swerve.setSpeedMultiplier(0.3),
			() -> {},
			(wasInterrupted) -> this.robot.swerve.setSpeedMultiplier(1),
			() -> false
		)));
		controller.b().whileTrue(this.robot.commands.makeToast());
		controller.y().whileTrue(robot.shootAtAngle(Degrees.of(38.5), 1));
		controller.a().whileTrue(
			this.robot.shooter.commands.shoot()
				.alongWith(this.robot.intake.commands.intake())
				.withTimeout(1)
		);
		
		return this;
		
	}
	
	/**
	 * Registers the right bumper of the given controller to hold the arm at the
	 * 'climbing angle' while held.
	 *
	 * @param controller The controller that should be registered to handle the
	 * given controls.
	 * @return This builder, for method chaining.
	 */
	public ControlsSchemeBuilder useRightBumperToClimb(
		CommandXboxController controller
	) {
		
		controller.rightBumper().whileTrue(
			this.robot.arm.commands.holdAtAngle(Degrees.of(100))
		);
		
		return this;
		
	}
	
}
