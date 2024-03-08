// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.CANDevice;
import frc.robot.constants.DoublePreference;
import frc.robot.constants.RobotDimensions;
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
	
	// private Odometry odometry;
	
	protected final SwerveDriveOdometry swerveDriveOdometry;
	
	protected final SwerveModulePosition[] modulePositions;
	
	protected final AHRS gyro;
	
	protected Supplier<Point> vxyInput;
	
	protected DoubleSupplier vThetaInput;
	
	protected final SwerveDriveKinematics kinematics;

	protected final HolonomicDriveController controller;
	
	protected boolean isHeadingLockEnabled;
	
	protected ChassisSpeeds currentRawChassisSpeeds;
	
	protected ChassisSpeeds currentActualChassisSpeeds;
	
	public final StartPosition startPosition;
	
	public final Swerve.Commands commands;
	
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
		
		this.headingPIDController = new PIDController(0.015, 0, 0);
		this.gyro = new AHRS();
		this.vxyInput = () -> new Point(0, 0);
		this.vThetaInput = () -> 0;
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
		swerveDriveOdometry = new SwerveDriveOdometry(
			kinematics,
			this.getFieldRelativeHeadingRotation2d(),
			modulePositions,
			new Pose2d(startPosition.translation, this.getFieldRelativeHeadingRotation2d())
		);
		
		controller = new HolonomicDriveController(
			new PIDController(0.01, 0, 0),
			new PIDController(0.01, 0, 0),
			new ProfiledPIDController(0.01, 0, 0, new Constraints(2 * Math.PI, .5))
		);

		// Create a new sendable field for each module
//		RobotContainer.putSendable("Analysis Tab", "fl-Module", frontLeftSwerveModule);
//		RobotContainer.putSendable("Analysis Tab", "fr-Module", frontRightSwerveModule);
//		RobotContainer.putSendable("Analysis Tab", "rl-Module", rearLeftSwerveModule);
//		RobotContainer.putSendable("Analysis Tab", "rr-Module", rearRightSwerveModule);
//		RobotContainer.putSendable("Analysis Tab", "gyro", gyro);
		// RobotContainer.putSendable("kinematics", odometry);
		
		this.isHeadingLockEnabled = true;
		this.currentRawChassisSpeeds = new ChassisSpeeds(0, 0, 0);
		this.currentActualChassisSpeeds = new ChassisSpeeds(0, 0, 0);
		this.commands = new Swerve.Commands();
		
		this.headingPIDController.enableContinuousInput(0, 360);
		
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
	
	/**
	 * Configures the DoubleSuppliers used by this Swerve instance to inform the
	 * relative movement of the robot.
	 *
	 * @param vxInput The DoubleSupplier that provides the x-axis velocity of
	 * the robot.
	 * @param vyInput The DoubleSupplier that provides the y-axis velocity of
	 * the robot.
	 * @param vThetaInput The DoubleSupplier that provides the angular velocity
	 * of the robot.
	 */
	public void configureChassisSpeedInputs(
		DoubleSupplier vxInput,
		DoubleSupplier vyInput,
		DoubleSupplier vThetaInput
	) {
		
		this.configureChassisSpeedInputs(
			() -> new Point(
				vxInput.getAsDouble(),
				vyInput.getAsDouble()
			),
			vThetaInput
		);
		
	}
	
	public void configureChassisSpeedInputs(
		Supplier<Point> vxyInput,
		DoubleSupplier vThetaInput
	) {
		
		this.vxyInput = vxyInput;
		this.vThetaInput = vThetaInput;
		
	}
	
	/**
	 * Runs the stop() method on each module
	 */
	public void stop() {
		
		this.getModuleStream().forEach(SwerveModule::stop);
		
	}
	
	public Pose2d updateOdometry() {
		
		return swerveDriveOdometry.update(
			this.getFieldRelativeHeadingRotation2d(),
			this.getWheelPositions()
		);
		
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
		
//		return new SwerveDriveWheelPositions(
//			(SwerveModulePosition[]) this.getModuleStream().toArray()
//		);
		
	}
	
	public void calibrateFieldRelativeHeading() {
		
		this.gyro.reset();
		this.headingPIDController.setSetpoint(
			this.getFieldRelativeHeading().in(Degrees)
		);
		
		this.swerveDriveOdometry.resetPosition(
			this.getFieldRelativeHeadingRotation2d(),
			modulePositions,
			this.updateOdometry()
		);
		
	}
	
	public StartPosition getStartPosition() {
		
		return startPosition;
		
	}
	
	public Measure<Angle> getFieldRelativeHeading() {
		
		return Degrees.of(
			this.getFieldRelativeHeadingRotation2d().getDegrees()
		);
		
	}
	
	public Rotation2d getFieldRelativeHeadingRotation2d() {
		
		return this.gyro.getRotation2d()/*.rotateBy(Rotation2d.fromDegrees(90))*/;
		
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
	
	public void driveFieldRelative(double x, double y, double rotation) {
		
		this.applyFieldRelativeChassisSpeeds(
			new ChassisSpeeds(x, y, rotation)
		);
		
	}
	
	public void driveRobotRelative(double x, double y, double rotation) {
		
		this.applyChassisSpeeds(
			new ChassisSpeeds(x, y, rotation)
		);
		
	}
	
	public void applyChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		
		this.currentRawChassisSpeeds = chassisSpeeds;
		
	}
	
	public void applyFieldRelativeChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		
		this.applyChassisSpeeds(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				chassisSpeeds,
				gyro.getRotation2d()
			)
		);
		
	}
	
	public void setFieldRelativeHeadingSetpoint(Measure<Angle> heading) {
		
		this.headingPIDController.setSetpoint(heading.in(Degrees));
		
	}
	
	public void xMode() {
		
		frontLeftSwerveModule.update(new SwerveModuleState(0, new Rotation2d(135)));
		frontRightSwerveModule.update(new SwerveModuleState(0, new Rotation2d(-135)));
		rearLeftSwerveModule.update(new SwerveModuleState(0, new Rotation2d(45)));
		rearRightSwerveModule.update(new SwerveModuleState(0, new Rotation2d(-45)));
		
	}
	
	@Override
	public void periodic() {
		
		double newAngularSetpoint = this.headingPIDController.getSetpoint() +
			(this.currentRawChassisSpeeds.omegaRadiansPerSecond * 2);
		
		double headingPIDOutput = this.headingPIDController.calculate(
			this.getFieldRelativeHeadingRotation2d().getDegrees(),
			newAngularSetpoint
		);
		
		Point vxy = this.vxyInput.get();
		double vTheta = this.vThetaInput.getAsDouble();
		
		this.currentRawChassisSpeeds = new ChassisSpeeds(
			vxy.x,
			vxy.y,
			vTheta * 2
		);
		
		this.currentActualChassisSpeeds = isHeadingLockEnabled ?
			new ChassisSpeeds(
				this.currentRawChassisSpeeds.vxMetersPerSecond,
				this.currentRawChassisSpeeds.vyMetersPerSecond,
				headingPIDOutput
			) : this.currentRawChassisSpeeds;
		
		this.currentActualChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			this.currentActualChassisSpeeds,
			this.getFieldRelativeHeadingRotation2d()
		);
		
		SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
			this.currentActualChassisSpeeds
		);
		
		this.frontLeftSwerveModule.update(moduleStates[0]);
		this.frontRightSwerveModule.update(moduleStates[1]);
		this.rearLeftSwerveModule.update(moduleStates[2]);
		this.rearRightSwerveModule.update(moduleStates[3]);
		
		updateOdometry();
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
	
//		builder.addDoubleProperty(
//			"P term",
//			this.frontLeftSwerveModule.steerPIDController::getP,
//			(kP) -> this.getModuleStream().forEach(
//				(module) -> module.steerPIDController.setP(kP)
//			)
//		);
//
//		builder.addDoubleProperty(
//			"D term",
//			this.frontLeftSwerveModule.steerPIDController::getD,
//			(kD) -> this.getModuleStream().forEach(
//				(module) -> module.steerPIDController.setD(kD)
//			)
//		);
//
//		builder.addDoubleProperty(
//			"Steering tolerance",
//			this.frontLeftSwerveModule.steerPIDController::getPositionTolerance,
//			(tolerance) -> this.getModuleStream().forEach(
//				(module) -> module.steerPIDController.setTolerance(tolerance)
//			)
//		);
//
//		builder.addDoubleProperty(
//			"Steering kS",
//			() -> this.frontLeftSwerveModule.steerFeedforward.ks,
//			(kS) -> this.getModuleStream().forEach(
//				(module) -> module.steerFeedforward = new SimpleMotorFeedforward(
//					kS,
//					module.steerFeedforward.kv
//				)
//			)
//		);

//		builder.addDoubleProperty("Swerve Odometry X Component", () -> updateOdometry().getX(), null);
//		builder.addDoubleProperty("Swerve Odometry Y Component", () -> updateOdometry().getY(), null);
		
	}
	
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
			
			return Swerve.this
				.runOnce(Swerve.this::resetOdometry)
				.withName("Calibrate Swerve Odometry")
				.ignoringDisable(true);
			
		}
		
		public Command setFieldRelativeHeading(Measure<Angle> heading) {
			
			return Swerve.this.runOnce(
				() -> Swerve.this.setFieldRelativeHeadingSetpoint(heading)
			);
			
		}
		
		public Command driveFieldRelative(Supplier<Point> xy, DoubleSupplier rotation) {
			
			return Swerve.this.run(() -> {
				
				Point xyPoint = xy.get();
				
				Swerve.this.driveFieldRelative(
					xyPoint.x,
					xyPoint.y,
					rotation.getAsDouble()
				);
				
			});
			
		}
		
		public Command driveRobotRelative(Supplier<Point> xy, DoubleSupplier rotation) {
			
			return Swerve.this.run(() -> {
				
				Point xyPoint = xy.get();
				
				Swerve.this.driveRobotRelative(
					xyPoint.x,
					xyPoint.y,
					rotation.getAsDouble()
				);
				
			});
			
		}
		
		public SwerveControllerCommand drive(Trajectory trajectory, Rotation2d rotation) {
			
			return new SwerveControllerCommand(
				trajectory,
				Swerve.this::getRobotPose,
				Swerve.this.kinematics,
				Swerve.this.controller,
				() -> rotation,
				(outputModuleStates) -> {},
				Swerve.this
			);
			
		}
		
	}
	
}
