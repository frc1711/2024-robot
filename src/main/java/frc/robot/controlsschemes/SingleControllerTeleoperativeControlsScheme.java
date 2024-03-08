package frc.robot.controlsschemes;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ComplexCommands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.DoubleSupplierBuilder;
import frc.robot.util.PointSupplierBuilder;

import static edu.wpi.first.units.Units.Degrees;

public class SingleControllerTeleoperativeControlsScheme implements ControlsScheme {
	
	protected static final double TRIGGER_THRESHOLD = 0.5;
	
	protected static final double JOYSTICK_DEADBAND = 0.1;
	
	protected static final double LINEAR_INPUT_SMOOTHING_POWER = 2;
	
	@Override
	public void configureControls(
		RobotContainer robotContainer,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		Swerve.Commands swerve = robotContainer.swerveSubsystem.commands;
		Intake.Commands intake = robotContainer.intake.commands;
		Shooter.Commands shooter = robotContainer.shooter.commands;
		Arm.Commands arm = robotContainer.arm.commands;
		
		robotContainer.arm.setDefaultCommand(
			arm.holdAtAngle(Degrees.of(0))
		);
		
		robotContainer.shooter.setDefaultCommand(new FunctionalCommand(
			robotContainer.shooter::stop,
			() -> {},
			(wasInterrupted) -> {},
			() -> false,
			robotContainer.shooter
		));
		
		robotContainer.intake.setDefaultCommand(new FunctionalCommand(
			robotContainer.intake::stop,
			() -> {},
			(wasInterrupted) -> {},
			() -> false,
			robotContainer.intake
		));
		
		controller1.x().whileTrue(ComplexCommands.shootAtAngle(
			robotContainer,
			Degrees.of(55),
			1
		));
		
		controller1.b().whileTrue(ComplexCommands.shootAtAngle(
			robotContainer,
			Degrees.of(98),
			0.12
		));
		
		controller1.y().whileTrue(ComplexCommands.shootAtAngle(
			robotContainer,
			Degrees.of(30),
			1
		));
		
		// Spin the intake outwards while the left trigger is pressed.
		controller1.leftTrigger(TRIGGER_THRESHOLD).whileTrue(intake.outtake().alongWith(shooter.shoot(-0.5)));
		
		// Spin the intake inwards while the right trigger is pressed.
		controller1.rightTrigger(TRIGGER_THRESHOLD).whileTrue(intake.intake());
		
		// Calibrate the field relative heading of the robot when the 'select'
		// ('back') button is pressed on controller #1.
		controller1.back().onTrue(swerve.calibrateFieldRelativeHeading());
		
		// Snap the robot to the cardinal field relative headings via the D-pad.
		controller1.povUp().onTrue(swerve.setFieldRelativeHeading(Degrees.of(0)));
		controller1.povLeft().onTrue(swerve.setFieldRelativeHeading(Degrees.of(90)));
		controller1.povDown().onTrue(swerve.setFieldRelativeHeading(Degrees.of(180)));
		controller1.povRight().onTrue(swerve.setFieldRelativeHeading(Degrees.of(270)));
		
		robotContainer.swerveSubsystem.setDefaultCommand(
			swerve.driveFieldRelative(
				PointSupplierBuilder.fromLeftJoystick(controller1)
					.normalizeXboxJoystickToNWU()
					.withClamp(-1, 1)
					.withScaledDeadband(JOYSTICK_DEADBAND)
					.withExponentialCurve(LINEAR_INPUT_SMOOTHING_POWER)
					.withScaling(0.6),
				DoubleSupplierBuilder.fromRightX(controller1)
					.withScaling(-1)
					.withClamp(-1, 1)
					.withScaledDeadband(JOYSTICK_DEADBAND)
					.withExponentialCurve(LINEAR_INPUT_SMOOTHING_POWER)
			)
		);
		
	}
	
}
