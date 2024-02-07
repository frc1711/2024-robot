// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.auton.framework.basic.OdometryAuton;
import frc.robot.commands.auton.framework.basic.SwerveAuton;
import frc.robot.commands.auton.framework.basic.timed.TimedSwerveAuton;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModule;
// import frc.robot.subsystems.swerve.Swerve.StartPosition;

public class RobotContainer {

	private final Swerve swerveSubsystem;
	private final SwerveModule flModule, frModule, rlModule, rrModule;
	private final AHRS gyro;
	private final DriveCommand driveCommand;
	private final XboxController driveController;
  	private final SendableChooser<Supplier<Command>> autonChooser;
	private final SendableChooser<Supplier<Swerve.StartPosition>> startPositionChooser;

  public RobotContainer() {
	driveController = new XboxController(0);
	startPositionChooser = new SendableChooser<>();
	configStartPositionChooser();

	flModule = new SwerveModule(IDMap.flSteerMotorID, IDMap.flDriveMotorID, IDMap.flEncoderID, new Translation2d(.43, .41)); //TODO: Update motor positions when using different robots
	frModule = new SwerveModule(IDMap.frSteerMotorID, IDMap.frDriveMotorID, IDMap.frEncoderID, new Translation2d(.43, -.41));
	rlModule = new SwerveModule(IDMap.rlSteerMotorID, IDMap.rlDriveMotorID, IDMap.rlEncoderID, new Translation2d(-.43, .41));
	rrModule = new SwerveModule(IDMap.rrSteerMotorID, IDMap.rrDriveMotorID, IDMap.rrEncoderID, new Translation2d(-.43, -.41));
	gyro = new AHRS();
	swerveSubsystem = new Swerve(flModule, frModule, rlModule, rrModule, gyro,
	startPositionChooser.getSelected().get()
	);
    autonChooser = new SendableChooser<>();

	driveCommand = new DriveCommand(
			swerveSubsystem, 
			() -> driveController.getLeftY(), 
			() -> driveController.getLeftX(), 
			() -> driveController.getRightX(), 
			() -> driveController.getRightTriggerAxis() > .1,
			() -> driveController.getRightStickButton(),
			() -> driveController.getXButton(),
			() -> driveController.getStartButton()
		);

	swerveSubsystem.setDefaultCommand(driveCommand);
    configAutonChooser();
  }

  /**
	 * Creates a new sendable field in the Analysis Tab of ShuffleBoard.
	 */
	public static void putSendable(String name, Sendable sendable) {
		
		Shuffleboard.getTab("Config Window").add(name, sendable);
		
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
			name,
			command.withName(name).ignoringDisable(canRunWhileDisabled)
		);

	}

	private void configAutonChooser() {
		autonChooser.addOption("Odometry Test Auton", () -> new OdometryAuton(swerveSubsystem, new Pose2d(new Translation2d(.5, 0), new Rotation2d()), 1));
		autonChooser.addOption("Timed Swerve Auton", () -> new TimedSwerveAuton(swerveSubsystem));

		putSendable("Auton Chooser", autonChooser);

	}

	private void configStartPositionChooser () {
		startPositionChooser.setDefaultOption("First Station Position", () -> Swerve.StartPosition.STATION_ONE);
		startPositionChooser.addOption("Second Station Position", () -> Swerve.StartPosition.STATION_TWO);
		startPositionChooser.addOption("Second Station Position", () -> Swerve.StartPosition.STATION_THREE);

		putSendable("Start Postion Chooser", startPositionChooser);
	}

  public Command getAutonomousCommand() {
    return autonChooser.getSelected().get();
  }
}
