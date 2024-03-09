package frc.robot.controlsschemes;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ComplexCommands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.DoubleSupplierBuilder;
import frc.robot.util.PointSupplierBuilder;

import java.util.Map;

import static edu.wpi.first.units.Units.Degrees;

public class ControlsSchemeBuilder {
	
	protected static final double TRIGGER_THRESHOLD = 0.5;
	
	protected static final double JOYSTICK_DEADBAND = 0.1;
	
	protected static final double LINEAR_INPUT_SMOOTHING_POWER = 2;
	
	protected final RobotContainer robot;
	
	public ControlsSchemeBuilder(RobotContainer robot) {
		
		this.robot = robot;
		
	}
	
	public ControlsSchemeBuilder configureDefaultRobotCommands() {
		
		Arm.Commands arm = this.robot.arm.commands;
		Shooter.Commands shooter = this.robot.shooter.commands;
		Intake.Commands intake = this.robot.intake.commands;
		
		this.robot.arm.setDefaultCommand(
			arm.holdAtAngle(Degrees.of(0))
		);
		
		this.robot.shooter.setDefaultCommand(shooter.stop());
		
		this.robot.intake.setDefaultCommand(intake.stop());
		
		return this;
		
	}
	
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
					.withExponentialCurve(LINEAR_INPUT_SMOOTHING_POWER)
					.withScaling(0.65),
				DoubleSupplierBuilder.fromRightX(controller)
					.withScaling(-1)
					.withClamp(-1, 1)
					.withScaledDeadband(JOYSTICK_DEADBAND)
					.withExponentialCurve(LINEAR_INPUT_SMOOTHING_POWER)
			)
		);
		
		return this;
		
	}
	
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
				Map.entry(false, intake.intake().until(
					this.robot.upperBeamBreakSensor::get
				))
			), this.robot.upperBeamBreakSensor::get)
		);
		
		return this;
		
	}
	
	public ControlsSchemeBuilder useABXYButtonsForShooting(
		CommandXboxController controller
	) {
		
		controller.x().whileTrue(ComplexCommands.shootAtAngle(
			robot,
			Degrees.of(55),
			1
		));
		
		controller.b().whileTrue(ComplexCommands.shootAtAngle(
			robot,
			Degrees.of(95),
			0.13
		));
		
		controller.y().whileTrue(ComplexCommands.shootAtAngle(
			robot,
			Degrees.of(40),
			1
		));
		
		return this;
		
	}
	
}
