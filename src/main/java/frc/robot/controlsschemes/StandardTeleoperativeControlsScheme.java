package frc.robot.controlsschemes;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ComplexCommands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.*;

import java.util.Map;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;

public class StandardTeleoperativeControlsScheme implements ControlsScheme {
	
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
		
		controller2.x().whileTrue(ComplexCommands.prepareToShootAtAngle(
			robotContainer.arm,
			robotContainer.shooter,
			Degrees.of(55),
			1
		).andThen(ComplexCommands.finishShootingAtAngle(
			robotContainer.arm,
			robotContainer.shooter,
			robotContainer.intake,
			Degrees.of(55),
			1
		)));
		
		controller2.b().whileTrue(ComplexCommands.prepareToShootAtAngle(
			robotContainer.arm,
			robotContainer.shooter,
			Degrees.of(98),
			0.12
		).andThen(ComplexCommands.finishShootingAtAngle(
			robotContainer.arm,
			robotContainer.shooter,
			robotContainer.intake,
			Degrees.of(100),
			0.12
		)));
		
		// Configure controls common to both controllers...
		Stream.of(controller1, controller2).forEach((controller) -> {
			
			// Spin the intake outwards while the left trigger is pressed.
			controller.leftTrigger(TRIGGER_THRESHOLD).whileTrue(
				intake.outtake().alongWith(shooter.shoot(-0.5))
			);
			
			// Spin the intake inwards while the right trigger is pressed.
			controller.rightTrigger(TRIGGER_THRESHOLD).whileTrue(new SelectCommand<Boolean>(Map.ofEntries(
				Map.entry(true, intake.intake()),
				Map.entry(false, intake.intake().until(robotContainer.upperBeamBreakSensor::get))
			), robotContainer.upperBeamBreakSensor::get));
			
		});
		
		controller1.back().onTrue(swerve.calibrateFieldRelativeHeading());
		
		controller1.povUp().onTrue(swerve.setFieldRelativeHeading(Degrees.of(0)));
		controller1.povLeft().onTrue(swerve.setFieldRelativeHeading(Degrees.of(90)));
		controller1.povDown().onTrue(swerve.setFieldRelativeHeading(Degrees.of(180)));
		controller1.povRight().onTrue(swerve.setFieldRelativeHeading(Degrees.of(270)));
		
		robotContainer.swerveSubsystem.configureChassisSpeedInputs(
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
		);
		
	}
	
}
