// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.CANDevice;
import frc.robot.configuration.DoublePreference;
import frc.robot.configuration.RobotDimensions;
import frc.robot.util.ControlsUtilities;
import frc.robot.util.Point;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;

public class Swerve extends SubsystemBase {
	
	protected final SwerveModule frontLeftSwerveModule;
	
	protected final SwerveModule frontRightSwerveModule;
	
	protected final SwerveModule rearLeftSwerveModule;
	
	protected final SwerveModule rearRightSwerveModule;
	
	protected final PIDController headingPIDController;
	
	protected final AHRS gyro;
	
	protected Measure<Angle> fieldRelativeHeadingAdjustmentAngle;
	
	protected final SwerveDriveKinematics kinematics;
	
	protected SwerveDriveOdometry odometry;
	
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
		
		this.headingPIDController = new PIDController(0, 0, 0);
		this.gyro = new AHRS();
		this.fieldRelativeHeadingAdjustmentAngle = Degrees.of(0);
		this.kinematics = new SwerveDriveKinematics(
			this.frontLeftSwerveModule.getPositionInRobot(),
			this.frontRightSwerveModule.getPositionInRobot(),
			this.rearLeftSwerveModule.getPositionInRobot(),
			this.rearRightSwerveModule.getPositionInRobot()
		);
		this.odometry = new SwerveDriveOdometry(
			this.kinematics,
			this.getFieldRelativeHeadingRotation2d(),
			this.getModulePositions()
		);

		// Create a new sendable field for each module
//		RobotContainer.putSendable("Analysis Tab", "fl-Module", frontLeftSwerveModule);
//		RobotContainer.putSendable("Analysis Tab", "fr-Module", frontRightSwerveModule);
//		RobotContainer.putSendable("Analysis Tab", "rl-Module", rearLeftSwerveModule);
//		RobotContainer.putSendable("Analysis Tab", "rr-Module", rearRightSwerveModule);
//		RobotContainer.putSendable("Analysis Tab", "gyro", gyro);
		// RobotContainer.putSendable("kinematics", odometry);
		
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

		shuffleboardCalibrationTab.add(
			this.commands.calibrateOdometry()
		);
		
	}
	
	protected Stream<SwerveModule> getModuleStream() {
		
		return Stream.of(
			frontLeftSwerveModule,
			frontRightSwerveModule,
			rearLeftSwerveModule,
			rearRightSwerveModule
		);
		
	}
	
	protected SwerveModulePosition[] getModulePositions() {
		
		return new SwerveModulePosition[] {
			this.frontLeftSwerveModule.getPosition(),
			this.frontRightSwerveModule.getPosition(),
			this.rearLeftSwerveModule.getPosition(),
			this.rearRightSwerveModule.getPosition()
		};
		
	}
	
	public HolonomicDriveController getHolonomicDriveController() {
		
		return new HolonomicDriveController(
			new PIDController(1, 0, 0),
			new PIDController(1, 0, 0),
			new ProfiledPIDController(
				0.015, 0, 0,
				new Constraints(30, 60)
			)
		);
		
	}
	
	public void stop() {
		
		this.applyChassisSpeeds(new ChassisSpeeds(0, 0, 0), false);
		this.getModuleStream().forEach(SwerveModule::stop);
		
	}
	
	public void calibrateFieldRelativeHeading() {
		
		this.calibrateFieldRelativeHeading(Degrees.of(0));

	}
	
	public void calibrateFieldRelativeHeading(Measure<Angle> currentHeading) {
		
		this.gyro.reset();
		this.fieldRelativeHeadingAdjustmentAngle = currentHeading.negate();
		this.headingPIDController.setSetpoint(
			this.getFieldRelativeHeading().in(Degrees)
		);

//		this.swerveDriveOdometry.resetPosition(
//			this.getFieldRelativeHeadingRotation2d(),
//			modulePositions,
//			this.updateOdometry()
//		);
		
	}
	
	public Measure<Angle> getFieldRelativeHeading() {
		
		return Degrees.of(
			this.getFieldRelativeHeadingRotation2d().getDegrees()
		);
		
	}
	
	public Rotation2d getFieldRelativeHeadingRotation2d() {
		
		Rotation2d adjustment = Rotation2d.fromDegrees(
			this.fieldRelativeHeadingAdjustmentAngle.in(Degrees)
		);
		
		return this.gyro.getRotation2d().plus(adjustment);
		
	}
	
	/**
	 * Runs the resetEncoder() method on each module
	 */
	public void calibrateModuleSteeringHeadings() {
		
		this.getModuleStream().forEach(
			(swerveModule) ->
				swerveModule.calibrateSteeringHeading(Degrees.of(0))
		);
		
	}
	
	public void applyChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
		
		if (fieldRelative) {
			
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				chassisSpeeds,
				this.getFieldRelativeHeadingRotation2d()
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
	
	public void xMode() {
		
		frontLeftSwerveModule.update(new SwerveModuleState(0, new Rotation2d(135)));
		frontRightSwerveModule.update(new SwerveModuleState(0, new Rotation2d(-135)));
		rearLeftSwerveModule.update(new SwerveModuleState(0, new Rotation2d(45)));
		rearRightSwerveModule.update(new SwerveModuleState(0, new Rotation2d(-45)));
		
	}
	
	@Override
	public void periodic() {
		
		double headingPIDOutput = this.headingPIDController.calculate(
			this.getFieldRelativeHeadingRotation2d().getDegrees()
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
		
		this.odometry.update(
			this.getFieldRelativeHeadingRotation2d(),
			this.getModulePositions()
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
			
			return Swerve.this
				.runOnce(Swerve.this::calibrateFieldRelativeHeading)
				.withName("Calibrate Swerve Field-relative Heading")
				.ignoringDisable(true);
			
		}
		
		public Command calibrateOdometry() {
			
			return new InstantCommand(() -> {});
//			return Swerve.this
//				.runOnce(Swerve.this::resetOdometry)
//				.withName("Calibrate Swerve Odometry")
//				.ignoringDisable(true);

		}
		
		public Command slowDown(double speedMultiplier) {
			
			return new FunctionalCommand(
				() -> Swerve.this.setSpeedMultiplier(speedMultiplier),
				() -> {},
				(ignored) -> Swerve.this.setSpeedMultiplier(1),
				() -> false,
				Swerve.this
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
