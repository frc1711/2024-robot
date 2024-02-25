// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DoublePreference;

public class SwerveModule extends SubsystemBase {
	
	CANcoder encoder;
	
	CANSparkMax driveMotor;
	
	CANSparkMax steerMotor;
	
	Translation2d motorMeters;
	
	PIDController steerPID = new PIDController(.01, 0, 0);
	
	RelativeEncoder driveEncoder;
	
	double unoptimizedRotation;
	
	double optimizedRotation;
	
	DoublePreference encoderOffsetPreference;
	
	double encoderOffset;
	
	// double encoderValue;
	
	double steerSpeed;
	
	double driveSpeed;
	
	double drivePercent;
	
	double initialVelocity = 0;
	
	/**
	 * Uses the average RPM of the motor, along with the circumference of the
	 * wheel, to calculate an approximate voltage value when given a speed in
	 * meters per second.
	 */
	private double maxSpeed = 11.5;
	
	double finalAngle;
	
	double regulatedAngle;
	
	boolean hasBeenEnabled = false;
	
	private CANSparkMax initializeMotor(int motorID) {
		
		CANSparkMax motor = new CANSparkMax(motorID, MotorType.kBrushless);
		motor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
		
		return motor;
		
	}
	
	// private CANCoder initializeEncoder(int encoderID) {
	// 	return new CANCoder(encoderID);
	// }
	
	public SwerveModule(
		int steerMotorID,
		int driveMotorID,
		int encoderID,
		DoublePreference encoderOffsetPreference,
		Translation2d motorMeters
	) {
		
		encoder = new CANcoder(encoderID);
		driveMotor = initializeMotor(driveMotorID);
		steerMotor = initializeMotor(steerMotorID);
		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(1 / 6.12);
		steerPID.enableContinuousInput(-180, 180);
		this.motorMeters = motorMeters;
		steerPID.setTolerance(1);
		this.encoderOffsetPreference = encoderOffsetPreference;
		this.setEncoderOffsetFromPreference();
		
	}
	
	public SwerveModulePosition getPosition() {
		
		return new SwerveModulePosition(
			driveEncoder.getPosition() * .1,
			getEncoderRotation()
		);
		
	}
	
	double distanceOffset = 0;
	
	public void resetModuleDistance() {
		
		distanceOffset = driveEncoder.getPosition() * .1;
		
	}
	
	private double metersPerSecondToPercentage(double metersPerSecond) {
		
		return metersPerSecond;
		
	}
	
	public void setEncoderOffset(double encoderOffset) {
		
		Preferences.setDouble(
			this.encoderOffsetPreference.key,
			encoderOffset
		);
		
		this.encoderOffset = encoderOffset;
		
	}
	
	public void setEncoderOffsetFromPreference() {
		
		this.encoderOffset = Preferences.getDouble(
			this.encoderOffsetPreference.key,
			this.encoderOffsetPreference.defaultValue
		);
		
	}
	
	/**
	 * Sets the encoderOffset to the current value of the CANcoder. This value
	 * is later used to set a new zero position for the encoder.
	 */
	public void resetEncoder() {
		
		this.setEncoderOffset(
			encoder.getAbsolutePosition().getValueAsDouble() * 360
		);
		
	}
	
	public double getDriveMotorPercentage() {
		
		return drivePercent;
		
	}
	
	/**
	 * Uses the encoder offset, which is set using the resetEncoders() method,
	 * to determine the current position of the CANcoder.
	 */
	public Rotation2d getEncoderRotation() {
		
		regulatedAngle = (encoder.getAbsolutePosition().getValueAsDouble() * 360) - encoderOffset;
		
		if (regulatedAngle < -180) regulatedAngle += 360;
		else if (regulatedAngle > 180) regulatedAngle -= 360;
		
		return Rotation2d.fromDegrees(regulatedAngle);
		
	}
	
	/**
	 * Takes in a SwerveModuleState, then uses a PID controller to calculate
	 * approximate values for the steerSpeed and the metersPerSecondToVoltage()
	 * method to calculate the driveVoltage.
	 * <p>
	 * WIP
	 */
	public void update(SwerveModuleState desiredState, double speedMultiplier) {
		
		unoptimizedRotation = desiredState.angle.getDegrees();
		
		SwerveModuleState optimizedState = SwerveModuleState.optimize(
			desiredState,
			getEncoderRotation()
		);
		
		optimizedRotation = optimizedState.angle.getDegrees();
		
		double encoderRotation = getEncoderRotation().getDegrees();
		double desiredRotation = optimizedState.angle.getDegrees();
		double angularTolerance = 5.0;
		
		if ((Math.abs(encoderRotation - desiredRotation) < angularTolerance)) {
			
			this.steerSpeed = 0;
			
		} else {
			
			this.steerSpeed = steerPID.calculate(
				encoderRotation,
				desiredRotation
			);
			
		}
		
		this.drivePercent = metersPerSecondToPercentage(
			optimizedState.speedMetersPerSecond * speedMultiplier
		);
		
		this.driveSpeed = optimizedState.speedMetersPerSecond;
		
		driveMotor.set(this.drivePercent);
		steerMotor.set(this.steerSpeed);
		
	}
	
	public void stop() {
		
		driveMotor.set(0);
		steerMotor.set(0);
		
	}
	
	@Override
	public void periodic() {
		
		// This method will be called once per scheduler run
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty(
			"Unoptimized Rotation",
			() -> unoptimizedRotation,
			null
		);
		
		builder.addDoubleProperty(
			"Optimized Rotation",
			() -> optimizedRotation,
			null
		);
		
		builder.addDoubleProperty(
			"Encoder Rotation (Degrees)",
			() -> getEncoderRotation().getDegrees(),
			null
		);
		
		builder.addDoubleProperty(
			"Steer Speed",
			() -> steerSpeed,
			null
		);
		
		builder.addDoubleProperty(
			"Drive Speed",
			() -> driveSpeed,
			null
		);
		
		builder.addDoubleProperty(
			"distance-Readout",
			() -> driveEncoder.getPosition() * .1 - distanceOffset,
			null
		);
		
		builder.addDoubleProperty(
			"distance-Offset",
			() -> distanceOffset,
			null
		);
		
	}
	
}
