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
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANDevice;
import frc.robot.RobotContainer;
import frc.robot.constants.DoublePreference;
import frc.robot.constants.RobotDimensions;
import frc.robot.util.Point;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;
// import frc.robot.util.Odometry;

public class Swerve extends SubsystemBase {
	
	protected final SwerveModule frontLeftSwerveModule;
	
	protected final SwerveModule frontRightSwerveModule;
	
	protected final SwerveModule rearLeftSwerveModule;
	
	protected final SwerveModule rearRightSwerveModule;
	
	// private Odometry odometry;
	
	protected final SwerveDriveOdometry swerveDriveOdometry;
	
	protected final SwerveModulePosition[] modulePositions;
	
	protected final AHRS gyro;
	
	protected final SwerveDriveKinematics kinematics;
	
	public final StartPosition startPosition;
	
	public final Swerve.Commands commands;
	
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
		
		// The following wheelbase calculations assume that the center of the
		// robot is the average (centerpoint) of the swerve modules, although
		// this is not necessarily the case, but is fine for the purposes of
		// this project.
		
		double swerveModuleXOffsetFromCenterInMeters =
			RobotDimensions.LENGTHWISE_WHEELBASE.in(Units.Meters) / 2;
		
		double swerveModuleYOffsetFromCenterInMeters =
			RobotDimensions.WIDTHWISE_WHEELBASE.in(Units.Meters) / 2;
		
		this.frontLeftSwerveModule = new SwerveModule(
			CANDevice.FRONT_LEFT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_LEFT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_LEFT_ENCODER.id,
			DoublePreference.FRONT_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
			new Translation2d(
				swerveModuleXOffsetFromCenterInMeters,
				swerveModuleYOffsetFromCenterInMeters
			)
		);
		
		this.frontRightSwerveModule = new SwerveModule(
			CANDevice.FRONT_RIGHT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_RIGHT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_RIGHT_ENCODER.id,
			DoublePreference.FRONT_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
			new Translation2d(
				swerveModuleXOffsetFromCenterInMeters,
				-swerveModuleYOffsetFromCenterInMeters
			)
		);
		
		this.rearLeftSwerveModule = new SwerveModule(
			CANDevice.REAR_LEFT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.REAR_LEFT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.REAR_LEFT_ENCODER.id,
			DoublePreference.REAR_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
			new Translation2d(
				-swerveModuleXOffsetFromCenterInMeters,
				swerveModuleYOffsetFromCenterInMeters
			)
		);
		
		this.rearRightSwerveModule = new SwerveModule(
			CANDevice.REAR_RIGHT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.REAR_RIGHT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.REAR_RIGHT_ENCODER.id,
			DoublePreference.REAR_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
			new Translation2d(
				-swerveModuleXOffsetFromCenterInMeters,
				-swerveModuleYOffsetFromCenterInMeters
			)
		);
		
		this.gyro = new AHRS();
		
		this.startPosition = startPosition;
		
		this.modulePositions = new SwerveModulePosition[] {
			frontLeftSwerveModule.getPosition(),
			frontRightSwerveModule.getPosition(),
			rearLeftSwerveModule.getPosition(),
			rearRightSwerveModule.getPosition()
		};
		
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
		
		this.commands = new Swerve.Commands();
		
	}
	
	protected Stream<SwerveModule> getModuleStream() {
		
		return Stream.of(
			frontLeftSwerveModule,
			frontRightSwerveModule,
			rearLeftSwerveModule,
			rearRightSwerveModule
		);
		
	}
	
	/**
	 * Runs the stop() method on each module
	 */
	public void stop() {
		
		this.getModuleStream().forEach(SwerveModule::stop);
		
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
		
		this.getModuleStream().forEach(SwerveModule::resetModuleDistance);
		
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
		
		this.getModuleStream().forEach(SwerveModule::resetEncoder);
		
	}
	
	public void drive(double x, double y, double rotation) {
		
		this.applyFieldRelativeChassisSpeeds(
			new ChassisSpeeds(x, y, rotation),
			1
		);
		
	}
	
	public void applyChassisSpeeds(ChassisSpeeds desiredVelocity, double speedMultiplier) {
		
		SwerveModuleState[] moduleStates =
			kinematics.toSwerveModuleStates(desiredVelocity);
		
		this.frontLeftSwerveModule.update(moduleStates[0], speedMultiplier);
		this.frontRightSwerveModule.update(moduleStates[1], speedMultiplier);
		this.rearLeftSwerveModule.update(moduleStates[2], speedMultiplier);
		this.rearRightSwerveModule.update(moduleStates[3], speedMultiplier);
		
	}
	
	public void applyFieldRelativeChassisSpeeds(ChassisSpeeds desiredVelocity, double speedMultiplier) {
		
		this.applyChassisSpeeds(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				desiredVelocity,
				gyro.getRotation2d()
			),
			speedMultiplier
		);
		
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
	
	public class Commands {
		
		public Command drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
			
			return Swerve.this.run(() -> Swerve.this.drive(
				x.getAsDouble(),
				y.getAsDouble(),
				rotation.getAsDouble()
			));
			
		}
		
		public Command drive(Supplier<Point> xy, DoubleSupplier rotation) {
			
			return Swerve.this.run(() -> {
				
				Point xyPoint = xy.get();
				
				Swerve.this.drive(xyPoint.x, xyPoint.y, rotation.getAsDouble());
				
			});
			
		}
		
	}
	
}
