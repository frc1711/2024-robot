// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auton.Fellowship;
import frc.robot.commands.auton.King;
import frc.robot.commands.auton.TwoTowers;
import frc.robot.commands.auton.framework.basic.OdometryAuton;
import frc.robot.commands.auton.framework.basic.SwerveAuton;
import frc.robot.commands.auton.framework.basic.timed.TimeBasedSwerveAuton;
import frc.robot.constants.DoublePreference;
import frc.robot.constants.DIODevice;
import frc.robot.controlsschemes.StandardTeleoperativeControlsScheme;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
	
	public final Swerve swerveSubsystem;
	
	public final Shooter shooter;
	
	public final Intake intake;
	
	public final Arm arm;
	
	public final DigitalInput upperBeamBreakSensor;
	
	public final DigitalInput lowerBeamBreakSensor;
	
	public final CommandXboxController driveController;
	
	public final CommandXboxController subsystemController;
	
	public final SendableChooser<Supplier<Command>> autonChooser;
	
	public final SendableChooser<Supplier<Command>> testChooser;
	
	public final SendableChooser<Supplier<Swerve.StartPosition>> startPositionChooser;
	
	public RobotContainer() {
		
		driveController = new CommandXboxController(0);
		subsystemController = new CommandXboxController(1);
		startPositionChooser = new SendableChooser<>();
		// configStartPositionChooser();
		
		swerveSubsystem = new Swerve(Swerve.StartPosition.STATION_ONE);
		shooter = new Shooter();
		intake = new Intake();
		arm = new Arm();
		
		this.upperBeamBreakSensor = new DigitalInput(
			DIODevice.INTAKE_UPPER_BEAM_BREAK_SENSOR.id
		);
		
		this.lowerBeamBreakSensor = new DigitalInput(
			DIODevice.INTAKE_LOWER_BEAM_BREAK_SENSOR.id
		);
		
		 Shuffleboard.getTab("Subsystems").add("Arm", arm);
		// Shuffleboard.getTab("Subsystems").add("Shooter", shooter);
		// Shuffleboard.getTab("Subsystems").add("Intake", intake);
		// Shuffleboard.getTab("Subsystems").add("Swerve", this.swerveSubsystem);
		
		autonChooser = new SendableChooser<>();
		testChooser = new SendableChooser<>();
		
		// configTestTab();
		configAutonChooser();
		
		Shuffleboard.getTab("Subsystems").addBoolean(
			"Upper Beam Break",
			this.upperBeamBreakSensor::get
		);
		
	}
	
	public void initTeleop() {
		
		(new StandardTeleoperativeControlsScheme()).configureControls(
			this,
			this.driveController,
			this.subsystemController
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
		
		autonChooser.addOption(
			"Fellowship of the Ring (Basic Auton)",
			() -> new Fellowship(swerveSubsystem, shooter, intake, arm)
		);

		autonChooser.addOption(
			"The Two Towers (Two-Piece Auton)", 
			() -> new TwoTowers(this, swerveSubsystem, intake, shooter, arm)
		);

		autonChooser.addOption(
			"Return of the King (Three-Piece Auton)", 
			() -> new King(swerveSubsystem, intake, shooter, arm, this)
		);

		autonChooser.addOption(
			"An Unexpected Journey (Distance Config Auton)", 
			() -> new SwerveAuton(
					swerveSubsystem, 
					DoublePreference.DISTANCE_CONFIG_AUTON_X_SPEED.get(), 
					DoublePreference.DISTANCE_CONFIG_AUTON_Y_SPEED.get(), 
					swerveSubsystem.getFieldRelativeHeadingRotation2d()
				).raceWith( 
				new WaitCommand(DoublePreference.DISTANCE_CONFIG_AUTON_TIME.get())
			)
		);
		
		putSendable("Pre-match Tab", "Auton Chooser", autonChooser);
		
	}
	
	// private void configStartPositionChooser() {
		
	// 	startPositionChooser.setDefaultOption("First Station Position", () -> Swerve.StartPosition.STATION_ONE);
	// 	startPositionChooser.addOption("Second Station Position", () -> Swerve.StartPosition.STATION_TWO);
	// 	startPositionChooser.addOption("Second Station Position", () -> Swerve.StartPosition.STATION_THREE);
		
	// 	putSendable("Pre-match Tab", "Start Postion Chooser", startPositionChooser);
		
	// }
	
	public Command getAutonomousCommand() {
		
		return autonChooser.getSelected().get();
		
	}
	
}
