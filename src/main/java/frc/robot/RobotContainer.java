// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.configuration.DIODevice;
import frc.robot.controlsschemes.ControlsScheme;
import frc.robot.controlsschemes.SingleControllerTeleoperativeControlsScheme;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

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
		this.controlsScheme = new SingleControllerTeleoperativeControlsScheme();
		
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
	
}
