// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SwerveModule extends SubsystemBase {
	
	CANcoder encoder;
	
	CANSparkMax driveMotor;
	
	CANSparkMax steerMotor;
	
	Translation2d motorMeters;
	
	PIDController steerPID = new PIDController(.01, 0, 0);

	Timer distanceTimer, distanceXTimer, distanceYTimer;

	RelativeEncoder driveEncoder;
	
	double unoptimizedRotation;
	
	double optimizedRotation;
	
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
		Translation2d motorMeters
	) {
		
		encoder = new CANcoder(encoderID);
		driveMotor = initializeMotor(driveMotorID);
		steerMotor = initializeMotor(steerMotorID);
		driveEncoder = driveMotor.getEncoder();
		steerPID.enableContinuousInput(-180, 180);
		this.motorMeters = motorMeters;
		steerPID.setTolerance(1);
	}

	public SwerveModulePosition getPosition () {
		return new SwerveModulePosition(driveEncoder.getPosition(), getEncoderRotation());
	}
	
	private double metersPerSecondToPercentage(double metersPerSecond) {
		return metersPerSecond;
		
	}
	/**
	 * Sets the encoderOffset to the current value of the CANcoder. This value
	 * is later used to set a new zero position for the encoder.
	 */
	public void resetEncoder() {
		
		encoderOffset = encoder.getAbsolutePosition().getValueAsDouble() * 360;
		
	}

	public double getDriveMotorPercentage () {
		return drivePercent;
	}
	
	/**
	 * Uses the encoder offset, which is set using the resetEncoders() method, 
	 * to determine the current position of the CANcoder.
	 */
	public Rotation2d getEncoderRotation() {
		
		regulatedAngle = (encoder.getAbsolutePosition().getValueAsDouble() * 360.) - encoderOffset;
		
		if (regulatedAngle < -180) regulatedAngle += 360;
		else if (regulatedAngle > 180) regulatedAngle -= 360;
		
		return Rotation2d.fromDegrees(regulatedAngle);
		
	}
	
	/**
	 * Takes in a SwerveModuleState, then uses a PID controller to calculate 
	 * approximate values for the steerSpeed and the metersPerSecondToVoltage() 
	 * method to calculate the driveVoltage.
	 * 
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
	
	private boolean firstEnable = true;
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty(
			"unop-rotation",
			() -> unoptimizedRotation, 
			null
		);
		
		builder.addDoubleProperty(
			"op-rotation",
			() -> optimizedRotation, 
			null
		);
		
		builder.addDoubleProperty(
			"actual-rotation",
			() -> getEncoderRotation().getDegrees(), 
			null
		);
		
		builder.addDoubleProperty(
			"steer-Speed",
			() -> steerSpeed, 
			null
		);
		
		builder.addDoubleProperty(
			"drive-Speed",
			() -> driveSpeed, 
			null
		);
		
		// builder.addDoubleProperty(
		// 	"drive-Percent",
		// 	() -> drivePercent, 
		// 	null
		// );

		// builder.addDoubleProperty(
		// 	"drive-Voltage", 
		// 	() -> driveMotor.getBusVoltage(), 
		// 	null
		// );
		
		// builder.addDoubleProperty(
		// 	"drive-Current", 
		// 	() -> driveMotor.getOutputCurrent(), 
		// 	null
		// );
		
		// builder.addDoubleProperty(
		// 	"drive-DutyCycle", 
		// 	() -> driveMotor.get(), 
		// 	null
		// );

		builder.addDoubleProperty(
			"Timer", 
			() -> distanceTimer.get(), 
			null);
	}
	
}
