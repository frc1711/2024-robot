// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.DoublePreference;
import frc.robot.devicewrappers.RaptorsSparkMaxNEO;
import frc.robot.util.ControlsUtilities;

import static edu.wpi.first.units.Units.*;

public class SwerveModule extends SubsystemBase {
	
	/**
	 * The units of the angular heading of the module.
	 */
	protected static final Unit<Angle> ANGULAR_HEADING_UNITS = Degrees;
	
	/**
	 * The desired speed of the steering motor, in degrees per second.
	 */
	protected static final Measure<Velocity<Angle>> STEER_SPEED = DegreesPerSecond.of(30);
	
	/**
	 * The motor controller that controls the steer motor of this module.
	 */
	protected final RaptorsSparkMaxNEO steerMotorController;
	
	/**
	 * The motor controller that controls the drive motor of this module.
	 */
	protected final RaptorsSparkMaxNEO driveMotorController;
	
	/**
	 * The absolute encoder that measures the current heading of this module.
	 */
	public final CANcoder steerEncoder;
	
	/**
	 * The relative encoder that measures the current distance of this module.
	 */
	protected final RelativeEncoder driveEncoder;
	
	/**
	 * The feedforward controller that calculates the voltage required to
	 * achieve a desired speed for the steering motor.
	 */
	public SimpleMotorFeedforward steerFeedforward;

//	protected final SimpleMotorFeedforward driveFeedforward;
	
	/**
	 * The PID controller that controls the steer heading of this module.
	 */
	protected final PIDController steerPIDController;
	
	/**
	 * The PID controller that controls the drive speed of this module.
	 */
	protected final PIDController drivePIDController;
	
	/**
	 * The preference that stores the current angular offset of the steering
	 * encoder for this module.
	 */
	protected final DoublePreference steeringEncoderOffsetPreference;
	
	/**
	 * Initializes a new SwerveModule with the given motor controllers and
	 * encoders.
	 *
	 * @param steerMotorControllerCANID The CAN ID of the motor controller that
	 * controls the steer motor of this module.
	 * @param driveMotorControllerCANID The CAN ID of the motor controller that
	 * controls the drive motor of this module.
	 * @param encoderCANID The CAN ID of the absolute encoder that measures the
	 * current heading of this module.
	 * @param encoderOffsetPreference The preference that stores the current
	 * angular offset of the steering encoder for this module.
	 */
	public SwerveModule(
		int steerMotorControllerCANID,
		int driveMotorControllerCANID,
		int encoderCANID,
		DoublePreference encoderOffsetPreference
	) {
		
		this.steerMotorController =
			new RaptorsSparkMaxNEO(steerMotorControllerCANID)
				.inBrakeMode()
				.withoutInvertedOutput()
				.withSmartCurrentLimit(Amps.of(40));
		
		this.driveMotorController =
			new RaptorsSparkMaxNEO(driveMotorControllerCANID)
				.inBrakeMode()
				.withInvertedOutput()
				.withSmartCurrentLimit(Amps.of(40));
		
		this.steerEncoder = new CANcoder(encoderCANID);
		this.driveEncoder = this.driveMotorController.getEncoder();
		
		// 0.005 volts/degree ~= 1.7 volts/rotation divided by 360 degrees/rotation
		this.steerFeedforward = new SimpleMotorFeedforward(0.5, 0.005);
//		this.driveFeedforward = new SimpleMotorFeedforward(0, 0);
		
		this.steerPIDController = new PIDController(
			DoublePreference.SWERVE_STEER_PID_KP.get(),
			0,
			DoublePreference.SWERVE_STEER_PID_KD.get()
		);
		
		this.drivePIDController = new PIDController(
			DoublePreference.SWERVE_DRIVE_PID_KP.get(),
			0,
			DoublePreference.SWERVE_DRIVE_PID_KD.get()
		);
		
		this.steeringEncoderOffsetPreference = encoderOffsetPreference;
		
		// Configure the steering encoder.
		
		this.driveEncoder.setPositionConversionFactor(1 / (2 * Math.PI));
		
		// Configure the steering PID controller.
		
		// This PID controller's reference is a measurement of the error between
		// the actual current steering heading vs the setpoint, so the setpoint
		// is always 0.
		this.steerPIDController.setSetpoint(0);
		this.steerPIDController.setTolerance(2);
		this.steerPIDController.enableContinuousInput(-180, 180);
		
	}
	
	public SwerveModulePosition getPosition() {
		
		return new SwerveModulePosition(
			driveEncoder.getPosition() * .1,
			new Rotation2d(this.getSteeringHeading())
		);
		
	}
	
	double distanceOffset = 0;
	
	public void resetModuleDistance() {
		
		distanceOffset = driveEncoder.getPosition() * .1;
		
	}
	
	/**
	 * Sets the current angular offset of the steering heading for this swerve
	 * module.
	 *
	 * @param offset The new angular offset of the steering heading for this
	 * swerve module.
	 */
	public void setSteeringEncoderOffset(Measure<Angle> offset) {
		
		this.steeringEncoderOffsetPreference.set(
			offset.in(SwerveModule.ANGULAR_HEADING_UNITS)
		);
		
	}
	
	/**
	 * Returns the current angular offset of the steering heading for this
	 * swerve module.
	 *
	 * @return The current angular offset of the steering heading for this
	 * swerve module.
	 */
	public Measure<Angle> getSteeringEncoderOffset() {
		
		return Degrees.of(this.steeringEncoderOffsetPreference.get());
		
	}
	
	/**
	 * Calibrates the steering heading for this swerve module such that it will
	 * return the given angle in its current physical position after this method
	 * is called.
	 *
	 * @param currentHeading The heading to calibrate this swerve module at.
	 */
	public void calibrateSteeringHeading(Measure<Angle> currentHeading) {
		
		this.setSteeringEncoderOffset(
			this.getSteeringEncoderOffset()
				.minus(this.getSteeringHeading().minus(currentHeading))
		);
		
	}
	
	/**
	 * Returns the current heading of this swerve module, oriented in the
	 * standard FRC 'northwest-up' ('NWU') coordinate system.
	 *
	 * @return The current heading of this swerve module.
	 */
	public Measure<Angle> getSteeringHeading() {
		
		// Get the raw heading value from the CANcoder.
		Measure<Angle> rawSteeringAngle = Rotations.of(
			this.steerEncoder.getAbsolutePosition().getValueAsDouble()
		);
		
		// Correct for the encoder offset.
		Measure<Angle> correctedSteeringAngle = rawSteeringAngle.plus(
			Degrees.of(this.steeringEncoderOffsetPreference.get())
		);
		
		// Normalize the steering angle value to the range [-180, 180].
		return Degrees.of(
			ControlsUtilities.normalizeToRange(
				correctedSteeringAngle.in(Degrees),
				-180,
				180
			)
		);
		
	}
	
	/**
	 * Returns a measurement of the error between the current steering heading
	 * and the setpoint of the steering PID controller.
	 *
	 * @return A measurement of the error between the current steering heading
	 * and the setpoint of the steering PID controller.
	 */
	public Measure<Angle> getSteeringHeadingError() {
		
		Measure<Angle> steerHeadingError =
			this.getSteeringHeading().minus(Degrees.of(
				this.steerPIDController.getSetpoint()
			));
		
		return Degrees.of(ControlsUtilities.normalizeToRange(
			steerHeadingError.in(Degrees),
			-180,
			180
		));
		
	}
	
	/**
	 * Takes in a SwerveModuleState, then uses a PID controller to calculate
	 * approximate values for the steerSpeed and the metersPerSecondToVoltage()
	 * method to calculate the driveVoltage.
	 * <p>
	 * WIP
	 */
	public void update(SwerveModuleState desiredState) {
		
		Measure<Angle> currentSteeringHeading = this.getSteeringHeading();
		
		SwerveModuleState optimizedState = SwerveModuleState.optimize(
			desiredState,
			new Rotation2d(currentSteeringHeading)
		);
		
		this.steerPIDController.setSetpoint(optimizedState.angle.getDegrees());
		this.drivePIDController.setSetpoint(
			optimizedState.speedMetersPerSecond
		);
		
	}
	
	@Override
	public void periodic() {

		double pidVoltage = this.steerPIDController.calculate(
			this.getSteeringHeading().in(Degrees)
		);

		if (steerPIDController.atSetpoint()) {

			this.steerMotorController.stopMotor();

		} else {

			double feedforwardVoltage = this.steerFeedforward.calculate(
				Math.copySign(
					SwerveModule.STEER_SPEED.in(DegreesPerSecond),
					pidVoltage
				)

			);

			this.steerMotorController.sparkMax.setVoltage(
				pidVoltage + feedforwardVoltage
			);

		}
		
		double driveSpeed = this.drivePIDController.getSetpoint();
		
		if (Math.abs(driveSpeed) > 0.05) {
			
			this.driveMotorController.set(
				Math.min(this.drivePIDController.getSetpoint(), 1)
			);
			
		} else this.driveMotorController.stopMotor();
		
	}
	
	/**
	 * Stops both the drive and steer motors of this swerve module.
	 */
	public void stop() {
		
		this.driveMotorController.stopMotor();
		this.steerMotorController.stopMotor();
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty(
			"heading",
			() -> this.getSteeringHeading().in(Degrees),
			null
		);
//
//		builder.addDoubleProperty(
//			"drive speed",
//			this.driveMotorController::get,
//			null
//		);
//
		builder.addDoubleProperty(
			"heading setpoint",
			this.steerPIDController::getSetpoint,
			this.steerPIDController::setSetpoint
		);
		
		builder.addDoubleProperty(
			"heading error",
			() -> this.getSteeringHeadingError().in(Degrees),
			null
		);
//
//		builder.addDoubleProperty(
//			"drive speed setpoint",
//			() -> this.drivePIDController.getSetpoint(),
//			null
//		);
//
//		builder.addDoubleProperty(
//			"Encoder Rotation (Degrees)",
//			() -> getEncoderRotation().getDegrees(),
//			null
//		);
//
//		builder.addDoubleProperty(
//			"Steer Speed",
//			() -> steerSpeed,
//			null
//		);
//
//		builder.addDoubleProperty(
//			"Drive Speed",
//			() -> driveSpeed,
//			null
//		);
//
//		builder.addDoubleProperty(
//			"distance-Readout",
//			() -> driveEncoder.getPosition() * .1 - distanceOffset,
//			null
//		);
//
//		builder.addDoubleProperty(
//			"distance-Offset",
//			() -> distanceOffset,
//			null
//		);
		
	}
	
}
