// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auton.Fellowship;
import frc.robot.commands.auton.King;
import frc.robot.commands.auton.TwoTowers;
import frc.robot.commands.auton.framework.basic.SwerveAuton;
import frc.robot.constants.DoublePreference;
import frc.robot.constants.DIODevice;
import frc.robot.controlsschemes.ControlsScheme;
import frc.robot.controlsschemes.StandardTeleoperativeControlsScheme;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.StartPosition;

public class RobotContainer {
	
	public final Swerve swerve;
	
	public final Shooter shooter;
	
	public final Intake intake;
	
	public final Arm arm;
	
	public final DigitalInput upperBeamBreakSensor;
	
	public final DigitalInput lowerBeamBreakSensor;
	
	public final CommandXboxController driveController;
	
	public final CommandXboxController subsystemController;
	
	protected final ControlsScheme controlsScheme;
	
	protected final SendableChooser<Supplier<Command>> autonChooser;
	
	public RobotContainer() {
		
		// Initialize the subsystems.
		this.swerve = new Swerve(Swerve.StartPosition.STATION_ONE);
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
		
		// Initialize the autonomous command chooser.
		this.autonChooser = RobotContainer.initializeAutonChooser(this);
		
		// Shuffleboard.getTab("Subsystems").add("Arm", arm);
		// Shuffleboard.getTab("Subsystems").add("Shooter", shooter);
		// Shuffleboard.getTab("Subsystems").add("Intake", intake);
		// Shuffleboard.getTab("Subsystems").add("Swerve", this.swerveSubsystem);
		
	}
	
	public void initTeleop() {
		
		this.controlsScheme.configureControls(
			this,
			this.driveController,
			this.subsystemController
		);
		
	}
	
	protected static SendableChooser<Supplier<Command>> initializeAutonChooser(
		RobotContainer robot
	) {
		
		SendableChooser<Supplier<Command>> autonChooser =
			new SendableChooser<>();
		
		autonChooser.addOption(
			"Fellowship of the Ring (Basic Auton)",
			() -> new Fellowship(robot, StartPosition.getSelectedStartPosition())
		);

		autonChooser.addOption(
			"The Two Towers (Two-Piece Auton)",
			() -> new TwoTowers(robot)
		);

		autonChooser.addOption(
			"Return of the King (Three-Piece Auton)",
			() -> new King(robot)
		);

		autonChooser.addOption(
			"An Unexpected Journey (Distance Config Auton)",
			() -> new SwerveAuton(
					robot,
					DoublePreference.DISTANCE_CONFIG_AUTON_X_SPEED.get(),
					DoublePreference.DISTANCE_CONFIG_AUTON_Y_SPEED.get(),
					robot.swerve.getFieldRelativeHeadingRotation2d()
				).raceWith(
				new WaitCommand(DoublePreference.DISTANCE_CONFIG_AUTON_TIME.get())
			)
		);
		
		Shuffleboard.getTab("Pre-match Tab")
			.add("Auton Chooser", autonChooser);
		
		return autonChooser;
		
	}
	
	public Optional<Command> getAutonomousCommand() {
		
		Supplier<Command> commandSupplier = autonChooser.getSelected();
		
		if (commandSupplier == null) return Optional.empty();
		
		Command command = commandSupplier.get();
		
		if (command == null) return Optional.empty();
		else return Optional.of(command);
		
	}
	
}
