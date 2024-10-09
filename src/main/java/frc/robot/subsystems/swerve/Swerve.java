// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.CANDevice;
import frc.robot.configuration.DoublePreference;
import frc.robot.configuration.RobotDimensions;
import frc.robot.devicewrappers.RaptorsNavX;
import frc.robot.devicewrappers.swerve.RaptorsSwerveModule;
import frc.robot.util.ControlsUtilities;
import frc.robot.util.Point;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;

public class Swerve extends SubsystemBase {
	
	protected static final int FRONT_LEFT_SWERVE_MODULE_ID = 0;
	
	protected static final int FRONT_RIGHT_SWERVE_MODULE_ID = 1;
	
	protected static final int REAR_LEFT_SWERVE_MODULE_ID = 2;
	
	protected static final int REAR_RIGHT_SWERVE_MODULE_ID = 3;
	
	protected final RaptorsSwerveModule frontLeftSwerveModule;
	
	protected final RaptorsSwerveModule frontRightSwerveModule;
	
	protected final RaptorsSwerveModule rearLeftSwerveModule;
	
	protected final RaptorsSwerveModule rearRightSwerveModule;
	
	protected final PIDController headingPIDController;
	
	protected final RaptorsNavX gyro;
	
	protected final SwerveDriveKinematics kinematics;
	
	protected boolean isHeadingLockEnabled;
	
	protected double speedMultiplier;
	
	protected ChassisSpeeds currentRawChassisSpeeds;
	
	protected ChassisSpeeds currentActualChassisSpeeds;
	
	public final Swerve.Commands commands;
	
	public Swerve() {
		
		// The following wheelbase calculations assume that the center of the
		// robot is the average (centerpoint) of the swerve modules, although
		// this is not necessarily the case, but is fine for the purposes of
		// this project.
		
		double swerveModuleXOffsetFromCenterInMeters =
			RobotDimensions.LENGTHWISE_WHEELBASE.in(Units.Meters) / 2;
		
		double swerveModuleYOffsetFromCenterInMeters =
			RobotDimensions.WIDTHWISE_WHEELBASE.in(Units.Meters) / 2;
		
		this.frontLeftSwerveModule = new RaptorsSwerveModule(
			CANDevice.FRONT_LEFT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_LEFT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_LEFT_ENCODER.id,
			DoublePreference.FRONT_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
			new Translation2d(
				swerveModuleXOffsetFromCenterInMeters,
				swerveModuleYOffsetFromCenterInMeters
			)
		);
		
		this.frontRightSwerveModule = new RaptorsSwerveModule(
			CANDevice.FRONT_RIGHT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_RIGHT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_RIGHT_ENCODER.id,
			DoublePreference.FRONT_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
			new Translation2d(
				swerveModuleXOffsetFromCenterInMeters,
				-swerveModuleYOffsetFromCenterInMeters
			)
		);
		
		this.rearLeftSwerveModule = new RaptorsSwerveModule(
			CANDevice.REAR_LEFT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.REAR_LEFT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.REAR_LEFT_ENCODER.id,
			DoublePreference.REAR_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
			new Translation2d(
				-swerveModuleXOffsetFromCenterInMeters,
				swerveModuleYOffsetFromCenterInMeters
			)
		);
		
		this.rearRightSwerveModule = new RaptorsSwerveModule(
			CANDevice.REAR_RIGHT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.REAR_RIGHT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.REAR_RIGHT_ENCODER.id,
			DoublePreference.REAR_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
			new Translation2d(
				-swerveModuleXOffsetFromCenterInMeters,
				-swerveModuleYOffsetFromCenterInMeters
			)
		);
		
		this.headingPIDController = new PIDController(0, 0, 0);
		this.gyro = new RaptorsNavX();
		this.kinematics = new SwerveDriveKinematics(
			this.frontLeftSwerveModule.getPositionInRobot(),
			this.frontRightSwerveModule.getPositionInRobot(),
			this.rearLeftSwerveModule.getPositionInRobot(),
			this.rearRightSwerveModule.getPositionInRobot()
		);
		
		this.isHeadingLockEnabled = true;
		this.speedMultiplier = 1;
		this.currentRawChassisSpeeds = new ChassisSpeeds(0, 0, 0);
		this.currentActualChassisSpeeds = new ChassisSpeeds(0, 0, 0);
		this.commands = new Swerve.Commands();
		
		this.headingPIDController.enableContinuousInput(0, 360);
		DoublePreference.SWERVE_HEADING_PID_KP
			.useValue(this.headingPIDController::setP);
		DoublePreference.SWERVE_HEADING_PID_KD
			.useValue(this.headingPIDController::setD);
		DoublePreference.SWERVE_HEADING_PID_TOLERANCE_DEGREES
			.useValue(this.headingPIDController::setTolerance);
		
		this.calibrateFieldRelativeHeading();
		
		ShuffleboardTab shuffleboardCalibrationTab =
			Shuffleboard.getTab("Calibration");

		shuffleboardCalibrationTab.add(
			this.commands.calibrateModuleSteeringHeadings()
		);

		shuffleboardCalibrationTab.add(
			this.commands.calibrateFieldRelativeHeading()
		);
		
	}
	
	protected Stream<RaptorsSwerveModule> getModuleStream() {
		
		return Stream.of(
			frontLeftSwerveModule,
			frontRightSwerveModule,
			rearLeftSwerveModule,
			rearRightSwerveModule
		);
		
	}
	
	public void stop() {
		
		this.applyChassisSpeeds(
			new ChassisSpeeds(
				0,
				0,
				0
			),
			false
		);
		this.getModuleStream().forEach(RaptorsSwerveModule::stop);
		
	}
	
	public void calibrateFieldRelativeHeading() {
		
		this.calibrateFieldRelativeHeading(Degrees.of(0));

	}
	
	public void calibrateFieldRelativeHeading(Measure<Angle> currentHeading) {
		
		this.gyro.calibrate(currentHeading);
		this.setFieldRelativeHeadingSetpoint(currentHeading.negate());
		
	}
	
	public Measure<Angle> getFieldRelativeHeading() {
		
		return this.gyro.getRotation();
		
	}
	
	/**
	 * Runs the resetEncoder() method on each module
	 */
	public void calibrateModuleSteeringHeadings() {
		
		this.getModuleStream().forEach(RaptorsSwerveModule::calibrateSteeringHeading);
		
	}
	
	public void applyChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
		
		if (fieldRelative) {
			
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				chassisSpeeds,
				Rotation2d.fromDegrees(this.gyro.getRotation().in(Degrees))
			);
			
		}
		
		// Poll the current state of the heading lock.
		boolean wasHeadingLockEnabled = this.isHeadingLockEnabled;
		
		// Enable the heading lock if we are not receiving any rotational input,
		// otherwise, disable it (if we *are* receiving rotational input).
		this.isHeadingLockEnabled = Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= 0;
		
		// Check for a rising edge of the heading lock state.
		boolean didHeadingLockBecomeEnabled = (
			!wasHeadingLockEnabled &&
			this.isHeadingLockEnabled
		);
		
		// If the heading lock *became* active...
		if (didHeadingLockBecomeEnabled) {
			
			// Update the heading setpoint to the heading we've rotated to while
			// the heading lock was disabled.
			this.headingPIDController.setSetpoint(
				this.getFieldRelativeHeading().in(Degrees)
			);
			
		}
		
		chassisSpeeds.omegaRadiansPerSecond *= 1.5;
		
		chassisSpeeds = chassisSpeeds.times(this.speedMultiplier);
		
		// Update the chassis speeds.
		this.currentRawChassisSpeeds = chassisSpeeds;
		
	}
	
	public void setFieldRelativeHeadingSetpoint(Measure<Angle> heading) {
		
		this.headingPIDController.setSetpoint(heading.in(Degrees));
		
	}
	
	public void setSpeedMultiplier(double speedMultiplier) {
		
		this.speedMultiplier = ControlsUtilities.applyClamp(
			speedMultiplier,
			0,
			1
		);
		
	}
	
	@Override
	public void periodic() {
		
		double headingPIDOutput = this.headingPIDController.calculate(
			this.getFieldRelativ/eHeading().in(Degrees)
		);
		
		this.currentActualChassisSpeeds = isHeadingLockEnabled ?
			new ChassisSpeeds(
				this.currentRawChassisSpeeds.vxMetersPerSecond,
				this.currentRawChassisSpeeds.vyMetersPerSecond,
				headingPIDOutput
			) : this.currentRawChassisSpeeds;
		
		SwerveModuleState[] moduleStates = this.kinematics.toSwerveModuleStates(
			this.currentActualChassisSpeeds
		);
		
		this.frontLeftSwerveModule.update(moduleStates[0]);
		this.frontRightSwerveModule.update(moduleStates[1]);
		this.rearLeftSwerveModule.update(moduleStates[2]);
		this.rearRightSwerveModule.update(moduleStates[3]);
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty(
			"Heading",
			() -> this.getFieldRelativeHeading().in(Degrees),
			(double headingDegrees) -> this.setFieldRelativeHeadingSetpoint(Degrees.of(headingDegrees))
		);
		
		builder.addDoubleProperty(
			"Heading Setpoint",
			() -> this.headingPIDController.getSetpoint(),
			(double headingDegrees) -> this.setFieldRelativeHeadingSetpoint(Degrees.of(headingDegrees))
		);
		
	}
	
	public class Commands {
		
		public Command calibrateModuleSteeringHeadings() {
			
			return Swerve.this
				.runOnce(Swerve.this::calibrateModuleSteeringHeadings)
				.withName("Calibrate Swerve Module Steering Headings")
				.ignoringDisable(true);
			
		}
		
		public Command calibrateFieldRelativeHeading() {
			
			return this.calibrateFieldRelativeHeading(Degrees.of(0));
			
		}
		
		public Command calibrateFieldRelativeHeading(Measure<Angle> currentHeading) {
			
			return Swerve.this
				.runOnce(() -> Swerve.this.calibrateFieldRelativeHeading(currentHeading))
				.withName("Calibrate Swerve Field-relative Heading")
				.ignoringDisable(true);
			
		}
		
		public Command slowDown(double speedMultiplier) {
			
			return new FunctionalCommand(
				() -> Swerve.this.setSpeedMultiplier(speedMultiplier),
				() -> {},
				(ignored) -> Swerve.this.setSpeedMultiplier(1),
				() -> false
			);
			
		}
		
		public Command slowDownWhile(double speedMultiplier, Command command) {
			
			return command.deadlineWith(this.slowDown(speedMultiplier));
			
		}
		
		public Command setFieldRelativeHeading(Measure<Angle> heading) {
			
			return Swerve.this.runOnce(
				() -> Swerve.this.setFieldRelativeHeadingSetpoint(heading)
			);
			
		}
		
		public Command driveFieldRelative(Supplier<Point> xy, DoubleSupplier rotation) {
			
			return Swerve.this.run(() -> {
				
				Point xyPoint = xy.get();
				
				Swerve.this.applyChassisSpeeds(
					new ChassisSpeeds(
						xyPoint.x,
						xyPoint.y,
						rotation.getAsDouble()
					),
					true
				);
				
			});
			
		}
		
		public Command driveRobotRelative(Supplier<Point> xy, DoubleSupplier rotation) {
			
			return Swerve.this.run(() -> {
				
				Point xyPoint = xy.get();
				
				Swerve.this.applyChassisSpeeds(
					new ChassisSpeeds(
						xyPoint.x,
						xyPoint.y,
						rotation.getAsDouble()
					),
					false
				);
				
			});
			
		}
		
		public Command driveForTime(
			Measure<Angle> translationAngle,
			double translationSpeed,
			Measure<Angle> heading,
			Measure<Time> duration
		) {
			
			return new FunctionalCommand(
				() -> Swerve.this.setFieldRelativeHeadingSetpoint(heading),
				() -> Swerve.this.applyChassisSpeeds(new ChassisSpeeds(
					Math.cos(translationAngle.in(Radians)) * translationSpeed,
					Math.sin(translationAngle.in(Radians)) * translationSpeed,
					0
				), true),
				(wasInterrupted) -> Swerve.this.stop(),
				() -> false,
				Swerve.this
			).withTimeout(duration.in(Seconds));
			
		}
		
		public Command driveForTime2(
			Measure<Angle> translationAngle,
			double translationSpeed,
			Measure<Angle> heading,
			Measure<Time> duration
		) {
			
			double rampTimeSeconds = 0.5;
			Timer timer = new Timer();
			
			return new FunctionalCommand(
				() -> {
					timer.start();
					Swerve.this.setFieldRelativeHeadingSetpoint(heading);
				},
				() -> {
					
					double timeSinceStart = timer.get();
					double timeUntilEnd = duration.in(Seconds) - timeSinceStart;
					double activeSpeed = Math.min(
						(timeSinceStart/rampTimeSeconds) * translationSpeed,
						(timeUntilEnd/rampTimeSeconds) * translationSpeed
					);
					
					activeSpeed = Math.min(activeSpeed, translationSpeed);
					
					Swerve.this.applyChassisSpeeds(new ChassisSpeeds(
						Math.cos(translationAngle.in(Radians)) * activeSpeed,
						Math.sin(translationAngle.in(Radians)) * activeSpeed,
						0
					), true);
					
				},
				(wasInterrupted) -> Swerve.this.stop(),
				() -> false,
				Swerve.this
			).withTimeout(duration.in(Seconds));
			
		}
		
//		public SwerveControllerCommand drive(Trajectory trajectory, Rotation2d rotation) {
//
//			return new SwerveControllerCommand(
//				trajectory,
//				Swerve.this::getRobotPose,
//				Swerve.this.kinematics,
//				Swerve.this.controller,
//				() -> rotation,
//				(outputModuleStates) -> {},
//				Swerve.this
//			);
//
//		}
		
	}
	
}
