// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auton.framework.basic.OdometryAuton;
import frc.robot.commands.auton.framework.basic.timed.TimedSwerveAuton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.ControlsTransformsUtilities;
import frc.robot.util.ControlsUtilities;
import frc.robot.util.Point;

public class RobotContainer {
	
	protected final Swerve swerveSubsystem;
	
	protected final Shooter shooter;
	
	protected final Intake intake;
	
	protected final Arm arm;
	
	protected final CommandXboxController driveController;
	
	protected final CommandXboxController subsystemController;
	
	protected final SendableChooser<Supplier<Command>> autonChooser;
	
	protected final SendableChooser<Supplier<Command>> testChooser;
	
	protected final SendableChooser<Supplier<Swerve.StartPosition>> startPositionChooser;
	
	public RobotContainer() {
		
		driveController = new CommandXboxController(0);
		subsystemController = new CommandXboxController(1);
		startPositionChooser = new SendableChooser<>();
		configStartPositionChooser();
		
		shooter = new Shooter();
		intake = new Intake();
		arm = new Arm();
		swerveSubsystem = new Swerve(startPositionChooser.getSelected().get());
		
		putSendable("Analysis Tab", "Odometry", swerveSubsystem);
		Shuffleboard.getTab("Subsystems").add("Arm", arm);
		Shuffleboard.getTab("Subsystems").add("Shooter", shooter);
		Shuffleboard.getTab("Subsystems").add("Intake", intake);
		
		autonChooser = new SendableChooser<>();
		testChooser = new SendableChooser<>();
		
		configTestTab();
		configAutonChooser();
		
	}
	
	public void initTeleop() {
		
		this.subsystemController.a().whileTrue(this.shooter.commands.shoot());
		this.subsystemController.y().whileTrue(this.intake.commands.intake());
		this.subsystemController.x().whileTrue(this.arm.commands.rotateToAngle(55));
		this.subsystemController.b().whileTrue(this.arm.commands.rotateToAngle(95));
		this.subsystemController.leftBumper().whileTrue(this.arm.commands.lowerArm());
		this.subsystemController.rightBumper().whileTrue(this.arm.commands.raiseArm());
		
		double intakeTriggerThreshold = 0.5;
		
		this.subsystemController.leftTrigger(intakeTriggerThreshold).whileTrue(this.intake.commands.outtake());
		this.subsystemController.rightTrigger(intakeTriggerThreshold).whileTrue(this.intake.commands.intake());
		this.driveController.leftTrigger(intakeTriggerThreshold).whileTrue(this.intake.commands.outtake());
		this.driveController.rightTrigger(intakeTriggerThreshold).whileTrue(this.intake.commands.intake());
		
		double deadband = 0.02;
		double power = 2;
		
		Supplier<Point> driveXYSupplier = () -> new Point(
			-ControlsUtilities.signedPower(this.driveController.getLeftY(), power),
			-ControlsUtilities.signedPower(this.driveController.getLeftX(), power)
		);
		
		driveXYSupplier = ControlsTransformsUtilities.applyCircularDeadband(
			driveXYSupplier,
			deadband
		);
		
		this.swerveSubsystem.setDefaultCommand(
			this.swerveSubsystem.commands.drive(
				driveXYSupplier,
				ControlsTransformsUtilities.signedPower(this.driveController::getRightX, power)
			)
		);
	
	}
	
	public Command getTestCommand() {
		
		return testChooser.getSelected().get();
		
	}
	
	/**
	 * Creates a new sendable field in the Analysis Tab of ShuffleBoard.
	 */
	public static void putSendable(String tab, String name, Sendable sendable) {
		
		Shuffleboard.getTab(tab).add(name, sendable);
		
	}
	
	/**
	 * Creates a new sendable command in the Analysis Tab of ShuffleBoard.
	 */
	public static void putCommand(
		String name,
		Command command,
		boolean canRunWhileDisabled
	) {
		
		putSendable(
			"Analysis Tab",
			name,
			command.withName(name).ignoringDisable(canRunWhileDisabled)
		);
		
	}
	
	private void configAutonChooser() {
		
		autonChooser.addOption("Odometry Test Auton", () -> new OdometryAuton(swerveSubsystem, new Pose2d(new Translation2d(swerveSubsystem.getStartPosition().getTranslation().getX() + .5, swerveSubsystem.getStartPosition().getTranslation().getY()), new Rotation2d(Math.PI)), .5));
		autonChooser.addOption("Timed Swerve Auton", () -> new TimedSwerveAuton(swerveSubsystem));
		
		putSendable("Pre-match Tab", "Auton Chooser", autonChooser);
		
	}
	
	private void configStartPositionChooser() {
		
		startPositionChooser.setDefaultOption("First Station Position", () -> Swerve.StartPosition.STATION_ONE);
		startPositionChooser.addOption("Second Station Position", () -> Swerve.StartPosition.STATION_TWO);
		startPositionChooser.addOption("Second Station Position", () -> Swerve.StartPosition.STATION_THREE);
		
		putSendable("Pre-match Tab", "Start Postion Chooser", startPositionChooser);
		
	}
	
	private void configTestTab() {
		
		// putSendable("Test Tab", "Shooter", shooter);
		//
		// testChooser.addOption(
		//  "Shooter Test Command",
		//  () -> new ShooterTest(
		// 	 shooter,
		// 	 driveController::getAButton,
		// 	 1,
		// 	 1
		//  )
		// );
		//
		// new TestCommand(
		//  shooter,
		//  new InstantCommand(
		// 	 () -> shooter.runShooter(
		// 		 1, 
		// 		 1
		// 	 ),
		// 	 shooter
		//  ),
		//  new InstantCommand(
		// 	 shooter::stop,
		// 	 shooter
		//  ),
		//  new InstantCommand(
		// 	 shooter::increaseShooterSpeed,
		// 	 shooter
		//  ),
		//  new InstantCommand(
		// 	 shooter::decreaseShooterSpeed,
		// 	 shooter
		//  ),
		//  driveController::getLeftBumperPressed,
		//  driveController::getRightBumperPressed,
		//  driveController::getXButtonPressed,
		//  driveController::getAButton
		// );
		//
		// testChooser.addOption(
		//  "Intake Test Command",
		//  () -> new TestCommand(
		// 	 shooter,
		// 	 new InstantCommand(
		// 		 () -> intake.runIntake(false),
		// 		 intake
		// 	 ),
		// 	 new InstantCommand(
		// 		 intake::stop,
		// 		 intake
		// 	 ),
		// 	 new InstantCommand(
		// 		 intake::increaseIntakeSpeed,
		// 		 intake
		// 	 ),
		// 	 new InstantCommand(
		// 		 intake::decreaseIntakeSpeed,
		// 		 intake
		// 	 ),
		// 	 driveController::getLeftBumperPressed,
		// 	 driveController::getRightBumperPressed,
		// 	 driveController::getAButtonPressed,
		// 	 driveController::getAButton
		//  )
		// );
		
		putSendable("Test Tab", "Test Command Chooser", testChooser);
		
	}
	
	public Command getAutonomousCommand() {
		
		return autonChooser.getSelected().get();
		
	}
	
}
