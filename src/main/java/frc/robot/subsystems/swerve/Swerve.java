// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.CANDevice;
// import frc.robot.util.Odometry;

public class Swerve extends SubsystemBase {
	
	private SwerveModule frontLeftSwerveModule;
	private SwerveModule frontRightSwerveModule;
	private SwerveModule rearLeftSwerveModule;
	private SwerveModule rearRightSwerveModule;
	
	// private Odometry odometry;
	
	private SwerveDriveOdometry swerveDriveOdometry;
	
	private SwerveModulePosition[] modulePositions;
	
	private AHRS gyro;
	
	private SwerveDriveKinematics kinematics;
	
	public StartPosition startPosition;
	
	// TODO: Find coordinates of start positions
	public enum StartPosition {
		
		STATION_ONE(new Translation2d(.8, 6.6), new Rotation2d(0)),
		STATION_TWO(new Translation2d(0, 0), new Rotation2d(0)),
		STATION_THREE(new Translation2d(0, 0), new Rotation2d(0));
		
		Translation2d translation;
		
		Rotation2d rotation;
		
		StartPosition(Translation2d translation, Rotation2d rotation) {
			
			this.translation = translation;
			this.rotation = rotation;
			
		}
		
		public Translation2d getTranslation() {
			
			return translation;
			
		}
		
	}
	
	public Swerve(StartPosition startPosition) {
		
		this.frontLeftSwerveModule = new SwerveModule(
			CANDevice.FRONT_LEFT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_LEFT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_LEFT_ENCODER.id,
			new Translation2d(.254, .269875)
		);
		
		this.frontRightSwerveModule = new SwerveModule(
			CANDevice.FRONT_RIGHT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_RIGHT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_RIGHT_ENCODER.id,
			new Translation2d(.254, -.269875)
		);
		
		this.rearLeftSwerveModule = new SwerveModule(
			CANDevice.REAR_LEFT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.REAR_LEFT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.REAR_LEFT_ENCODER.id,
			new Translation2d(-.254, .296875)
		);
		
		this.rearRightSwerveModule = new SwerveModule(
			CANDevice.REAR_RIGHT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.REAR_RIGHT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.REAR_RIGHT_ENCODER.id,
			new Translation2d(-.254, -.269875)
		);
		
		this.gyro = new AHRS();;
		
		this.startPosition = startPosition;
		modulePositions = new SwerveModulePosition[4];
		modulePositions[0] = frontLeftSwerveModule.getPosition();
		modulePositions[1] = frontRightSwerveModule.getPosition();
		modulePositions[2] = rearLeftSwerveModule.getPosition();
		modulePositions[3] = rearRightSwerveModule.getPosition();
		// odometry = new Odometry(gyro, Odometry.StartPosition.STATION_ONE);
		kinematics = new SwerveDriveKinematics(
			frontLeftSwerveModule.motorMeters,
			frontRightSwerveModule.motorMeters,
			rearLeftSwerveModule.motorMeters,
			rearRightSwerveModule.motorMeters
		);
		swerveDriveOdometry = new SwerveDriveOdometry(kinematics, getGyroRotation(), modulePositions, new Pose2d(startPosition.translation, getGyroRotation()));
		
		// Create a new sendable field for each module
		RobotContainer.putSendable("Analysis Tab", "fl-Module", frontLeftSwerveModule);
		RobotContainer.putSendable("Analysis Tab", "fr-Module", frontRightSwerveModule);
		RobotContainer.putSendable("Analysis Tab", "rl-Module", rearLeftSwerveModule);
		RobotContainer.putSendable("Analysis Tab", "rr-Module", rearRightSwerveModule);
		RobotContainer.putSendable("Analysis Tab", "gyro", gyro);
		// RobotContainer.putSendable("kinematics", odometry);
		
		// Create a new sendable command to reset the encoders
		RobotContainer.putCommand("Reset Encoders", new InstantCommand(this::resetEncoders, this), true);
		RobotContainer.putCommand("Reset Gyro", new InstantCommand(this::resetGyro, this), true);
		RobotContainer.putCommand("Reset Odometry", new InstantCommand(this::resetOdometry, this), true);
	}
	
	/**
	 * Runs the stop() method on each module
	 */
	public void stop() {
		
		frontLeftSwerveModule.stop();
		frontRightSwerveModule.stop();
		rearLeftSwerveModule.stop();
		rearRightSwerveModule.stop();
		
	}
	
	public Pose2d updateOdometry() {
		
		modulePositions[0] = frontLeftSwerveModule.getPosition();
		modulePositions[1] = frontRightSwerveModule.getPosition();
		modulePositions[2] = rearLeftSwerveModule.getPosition();
		modulePositions[3] = rearRightSwerveModule.getPosition();
		
		return swerveDriveOdometry.update(getGyroRotation(), new SwerveDriveWheelPositions(modulePositions));
		
	}
	
	public Pose2d getRobotPose() {
		
		return swerveDriveOdometry.getPoseMeters();
		
	}
	
	public void resetOdometry() {
		
		frontLeftSwerveModule.resetModuleDistance();
		frontRightSwerveModule.resetModuleDistance();
		rearLeftSwerveModule.resetModuleDistance();
		rearRightSwerveModule.resetModuleDistance();
		
	}
	
	public SwerveDriveWheelPositions getWheelPositions() {
		
		modulePositions[0] = frontLeftSwerveModule.getPosition();
		modulePositions[1] = frontRightSwerveModule.getPosition();
		modulePositions[2] = rearLeftSwerveModule.getPosition();
		modulePositions[3] = rearRightSwerveModule.getPosition();
		return new SwerveDriveWheelPositions(modulePositions);
		
	}
	
	public void resetGyro() {
		
		gyro.reset();
		swerveDriveOdometry.resetPosition(getGyroRotation(), modulePositions, updateOdometry());
		
	}
	
	public StartPosition getStartPosition() {
		
		return startPosition;
		
	}
	
	public Rotation2d getGyroRotation() {
		
		return gyro.getRotation2d();
		
	}
	
	public float getGyroPitch() {
		
		return gyro.getPitch();
		
	}
	
	/**
	 * Runs the resetEncoder() method on each module
	 */
	public void resetEncoders() {
		
		frontLeftSwerveModule.resetEncoder();
		frontRightSwerveModule.resetEncoder();
		rearLeftSwerveModule.resetEncoder();
		rearRightSwerveModule.resetEncoder();
		
	}
	
	public void updateModulesFieldRelative(ChassisSpeeds desiredVelocity, double speedMultiplier) {
		
		desiredVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(desiredVelocity, gyro.getRotation2d());
		SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(desiredVelocity);
		frontLeftSwerveModule.update(moduleStates[0], speedMultiplier);
		frontRightSwerveModule.update(moduleStates[1], speedMultiplier);
		rearLeftSwerveModule.update(moduleStates[2], speedMultiplier);
		rearRightSwerveModule.update(moduleStates[3], speedMultiplier);
		
	}
	
	/**
	 * Updates each module using the reverse kinematics feature from SwerveDriveKinematics
	 */
	public void updateModules(ChassisSpeeds desiredVelocity, double speedMultiplier) {
		
		SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(desiredVelocity);
		frontLeftSwerveModule.update(moduleStates[0], speedMultiplier);
		frontRightSwerveModule.update(moduleStates[1], speedMultiplier);
		rearLeftSwerveModule.update(moduleStates[2], speedMultiplier);
		rearRightSwerveModule.update(moduleStates[3], speedMultiplier);
		
	}
	
	public void xMode() {
		
		frontLeftSwerveModule.update(new SwerveModuleState(0, new Rotation2d(135)), 1);
		frontRightSwerveModule.update(new SwerveModuleState(0, new Rotation2d(-135)), 1);
		rearLeftSwerveModule.update(new SwerveModuleState(0, new Rotation2d(45)), 1);
		rearRightSwerveModule.update(new SwerveModuleState(0, new Rotation2d(-45)), 1);
		
	}
	
	@Override
	public void periodic() {
		
		updateOdometry();
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty("Swerve Odometry X Component", () -> updateOdometry().getX(), null);
		builder.addDoubleProperty("Swerve Odometry Y Component", () -> updateOdometry().getY(), null);
		
	}
	
}
