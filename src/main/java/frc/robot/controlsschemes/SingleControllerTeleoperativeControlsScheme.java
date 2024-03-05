package frc.robot.controlsschemes;

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
		
		// Spin up the shooter while the 'A' button is pressed on controller #2.
		controller1.a().whileTrue(shooter.shoot());
		
//		robotContainer.arm.setDefaultCommand(
//			arm.holdAtAngle(Degrees.of(0))
//		);
		
		controller1.x().onTrue(ComplexCommands.prepareToShootAtAngle(
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
		
		controller1.b().onTrue(ComplexCommands.prepareToShootAtAngle(
			robotContainer.arm,
			robotContainer.shooter,
			Degrees.of(100),
			0.12
		).andThen(ComplexCommands.finishShootingAtAngle(
			robotContainer.arm,
			robotContainer.shooter,
			robotContainer.intake,
			Degrees.of(100),
			0.12
		)));
		
//		controller1.b().onFalse(ComplexCommands.finishShootingAtAngle(
//			robotContainer.arm,
//			robotContainer.shooter,
//			robotContainer.intake,
//			Degrees.of(55),
//			1
//		));
		
//		controller2.b().whileTrue(
//			arm.holdAtAngle(Degrees.of(100))
//				.alongWith(
//					shooter.spinUp(),
//					(new WaitCommand(1)).andThen(intake.intake()).raceWith(new WaitCommand(1))
//				).andThen(shooter::stop).handleInterrupt(shooter::stop));
//		);
		
		// Move the arm towards the amp shooting angle while the 'B' button is
		// pressed on controller #2.
//		controller1.b().whileTrue(arm.holdAtAngle(Degrees.of(95)));
		
		// Move the arm towards the speaker shooting angle while the 'X' button
		// is pressed on controller #2.
//		controller1.x().whileTrue(arm.holdAtAngle(Degrees.of(55)));
		
		controller1.y().whileTrue(shooter.shoot(0.12));
		
		// Spin the intake outwards while the left trigger is pressed.
		controller1.leftTrigger(TRIGGER_THRESHOLD).whileTrue(intake.outtake().alongWith(shooter.shoot(-0.5)));
		
		// Spin the intake inwards while the right trigger is pressed.
		controller1.rightTrigger(TRIGGER_THRESHOLD).whileTrue(intake.intake());
		
//		controller1.leftBumper().whileTrue(robotContainer.arm.sysIdDynamicVoltageTest(SysIdRoutine.Direction.kReverse));
//		controller1.rightBumper().whileTrue(robotContainer.arm.sysIdDynamicVoltageTest(SysIdRoutine.Direction.kForward));
		
		controller1.back().onTrue(swerve.resetGyro());
		
		robotContainer.swerveSubsystem.setDefaultCommand(
			swerve.driveFieldRelative(
				PointSupplierBuilder.fromLeftJoystick(controller1)
					.normalizeXboxJoystickToNWU()
					.withClamp(-1, 1)
					.withScaledDeadband(JOYSTICK_DEADBAND)
					.withExponentialCurve(LINEAR_INPUT_SMOOTHING_POWER)
					.withScaling(0.5),
				DoubleSupplierBuilder.fromRightX(controller1)
					.withScaling(-1)
					.withClamp(-1, 1)
					.withScaledDeadband(JOYSTICK_DEADBAND)
					.withExponentialCurve(LINEAR_INPUT_SMOOTHING_POWER)
			)
		);
		
	}
	
}
