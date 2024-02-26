// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.constants.CANDevice;
import frc.robot.constants.DIODevice;
import frc.robot.constants.DoublePreference;
import frc.robot.util.ControlsUtilities;

import java.util.stream.Stream;

public class Arm extends PIDSubsystem {
	
	protected static final double ENCODER_POSITION_CONVERSION_FACTOR = 360;
	
	protected static final double ENCODER_CALIBRATION_ANGLE = 90;
	
	protected static final double MIN_ARM_ANGLE = 30;
	
	protected static final double MAX_ARM_ANGLE = 95;
	
	protected final CANSparkMax leftMotorController;

	protected final CANSparkMax rightMotorController;

	protected final DigitalInput leftUpperLimitSwitch;

	protected final SparkAbsoluteEncoder leftEncoder;

	protected final SparkAbsoluteEncoder rightEncoder;

//	protected final DigitalInput rightUpperLimitSwitch;

//	protected final DigitalInput leftLowerLimitSwitch;

//	protected final DigitalInput rightLowerLimitSwitch;
	
	public final Arm.Commands commands;
	
	public Arm() {
		
		super(new PIDController(0.15, 0, 0.05));
		
		this.getController().setTolerance(3);

		this.leftMotorController = new CANSparkMax(
			CANDevice.LEFT_PIVOT_MOTOR_CONTROLLER.id,
			MotorType.kBrushless
		);

		this.rightMotorController = new CANSparkMax(
			CANDevice.RIGHT_PIVOT_MOTOR_CONTROLLER.id,
			MotorType.kBrushless
		);
		
		this.leftMotorController.restoreFactoryDefaults();
		this.rightMotorController.restoreFactoryDefaults();
		
		this.leftEncoder = leftMotorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
		this.rightEncoder = rightMotorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

		leftUpperLimitSwitch = new DigitalInput(DIODevice.LEFT_UPPER_ARM_LIMIT_SWITCH.id);
//		rightUpperLimitSwitch = new DigitalInput(DIODevice.RIGHT_UPPER_ARM_LIMIT_SWITCH.id);
//		leftLowerLimitSwitch = new DigitalInput(DIODevice.LEFT_LOWER_ARM_LIMIT_SWITCH.id);
//		rightLowerLimitSwitch = new DigitalInput(DIODevice.RIGHT_LOWER_ARM_LIMIT_SWITCH.id);
		
		this.leftMotorController.setInverted(false);
		this.rightMotorController.setInverted(true);
		
		this.streamMotorControllers().forEach((motorController) -> {
			
			motorController.setOpenLoopRampRate(0.5);
			motorController.setIdleMode(CANSparkBase.IdleMode.kBrake);
			
//			motorController.enableSoftLimit(
//				CANSparkBase.SoftLimitDirection.kReverse,
//				true
//			);
//
//			motorController.enableSoftLimit(
//				CANSparkBase.SoftLimitDirection.kForward,
//				true
//			);
//
//			motorController.setSoftLimit(
//				CANSparkBase.SoftLimitDirection.kReverse,
//				(float) Arm.MIN_ARM_ANGLE
//			);
//
//			motorController.setSoftLimit(
//				CANSparkBase.SoftLimitDirection.kForward,
//				(float) Arm.MAX_ARM_ANGLE
//			);
			
		});
		
		this.leftEncoder.setInverted(false);
		this.rightEncoder.setInverted(true);
		
		this.streamEncoders().forEach((encoder) -> {
			
			encoder.setPositionConversionFactor(
				Arm.ENCODER_POSITION_CONVERSION_FACTOR
			);
			
		});
		
		DoublePreference leftEncoderZeroOffsetPreference = DoublePreference.ARM_LEFT_ENCODER_ZERO_OFFSET;
		DoublePreference rightEncoderZeroOffsetPreference = DoublePreference.ARM_RIGHT_ENCODER_ZERO_OFFSET;
		
		this.leftEncoder.setZeroOffset(Preferences.getDouble(
			leftEncoderZeroOffsetPreference.key,
			leftEncoderZeroOffsetPreference.defaultValue
		));
		
		this.rightEncoder.setZeroOffset(Preferences.getDouble(
			rightEncoderZeroOffsetPreference.key,
			rightEncoderZeroOffsetPreference.defaultValue
		));
		
		this.commands = new Arm.Commands();
		
		Shuffleboard.getTab("Subsystems").add(this.commands.calibrateArm());
		
	}
	
	@Override
	protected double getMeasurement() {
		
		return this.getAngle();
		
	}
	
	@Override
	protected void useOutput(double output, double setpoint) {
		
		double currentAngle = this.getAngle();
		boolean wouldOverrun = (
			(output > 0 && currentAngle > Arm.MAX_ARM_ANGLE) ||
			(output < 0 && currentAngle < Arm.MIN_ARM_ANGLE)
		);
		double effectiveOutput = wouldOverrun ? 0 : output;
		
		this.streamMotorControllers().forEach(
			(motorController) -> motorController.setVoltage(effectiveOutput)
		);
	
	}
	
	protected Stream<CANSparkMax> streamMotorControllers() {

		return Stream.of(this.leftMotorController, this.rightMotorController);

	}

	protected Stream<SparkAbsoluteEncoder> streamEncoders() {

		return Stream.of(this.leftEncoder, this.rightEncoder);

	}
	
	protected static void calibrateEncoder(
		SparkAbsoluteEncoder encoder,
		DoublePreference zeroOffsetPreference
	) {
		
		// Calculate the new zero offset relative to the existing one.
		double newZeroOffset = encoder.getZeroOffset() + encoder.getPosition();
		
		// Subtract the angle at which the arm is calibrated.
		newZeroOffset -= Arm.ENCODER_CALIBRATION_ANGLE;
		
		newZeroOffset = ControlsUtilities.normalizeToRange(
			newZeroOffset,
			0,
			Arm.ENCODER_POSITION_CONVERSION_FACTOR
		);
		
		Preferences.setDouble(zeroOffsetPreference.key, newZeroOffset);
		
		encoder.setZeroOffset(newZeroOffset);
		
	}
	
	/**
	 * Resets the 'zero angle' of the pivot encoders.
	 *
	 * Should be used when the arm is level with the chassis of the robot.
	 */
	public void calibrateArm() {
		
		Arm.calibrateEncoder(
			this.leftEncoder,
			DoublePreference.ARM_LEFT_ENCODER_ZERO_OFFSET
		);
		
		Arm.calibrateEncoder(
			this.rightEncoder,
			DoublePreference.ARM_RIGHT_ENCODER_ZERO_OFFSET
		);
		
	}
	
	public boolean areUpperLimitsSwitchesTripped() {
		
		return this.leftUpperLimitSwitch.get();
		
		// return (
		// 	this.leftUpperLimitSwitch.get() ||
		// 	this.rightUpperLimitSwitch.get()
		// );
		
	}
	
	public boolean areLowerLimitsSwitchesTripped() {
		
		return false;
		
		// return (
		// 	this.leftLowerLimitSwitch.get() ||
		// 	this.rightLowerLimitSwitch.get()
		// );
		
	}
	
	public boolean areLimitSwitchesTripped() {
		
		return (
			this.areUpperLimitsSwitchesTripped() ||
			this.areLowerLimitsSwitchesTripped()
		);
		
	}
	
	public boolean isArmOutsideUpperAngularLimit() {
		
		return this.getAngle() > Arm.MAX_ARM_ANGLE;
		
	}
	
	public boolean isArmOutsideLowerAngularLimit() {
		
		return this.getAngle() < Arm.MIN_ARM_ANGLE;
		
	}
	
	public boolean isArmOutsideAngularLimits() {
		
		return (
			this.isArmOutsideLowerAngularLimit() ||
			this.isArmOutsideUpperAngularLimit()
		);
		
	}
	
	public boolean isArmOutsideUpperLimit() {
		
		return (
			this.areUpperLimitsSwitchesTripped() ||
			this.isArmOutsideUpperAngularLimit()
		);
		
	}
	
	public boolean isArmOutsideLowerLimit() {
		
		return (
			this.areLowerLimitsSwitchesTripped() ||
			this.isArmOutsideLowerAngularLimit()
		);
		
	}
	
	public boolean isArmOutsideLimits() {
		
		return (
			this.isArmOutsideLowerLimit() ||
			this.isArmOutsideUpperLimit()
		);
		
	}
	
	public boolean isArmInsideLimits() {
		
		return !this.isArmOutsideLimits();
		
	}
	
	public void stop() {

		this.disable();
		this.streamMotorControllers().forEach(CANSparkMax::stopMotor);
		
	}
	
	public double getAngle() {

		return this.streamEncoders()
			.mapToDouble(SparkAbsoluteEncoder::getPosition)
			.average()
			.orElse(0);
        
	}
	
	public void rotate(boolean shouldRaise) {
		
		DoublePreference preference = DoublePreference.ARM_SPEED;
		double speed = Preferences.getDouble(preference.key, preference.defaultValue);
		
		this.rotate(shouldRaise ? speed : -speed);
		
	}

	public void rotate(double speed) {
		
		this.streamMotorControllers().forEach(
			(motorController) -> motorController.set(speed)
		);

	}
	
	public void rotateToAngle(double degrees) {
		
		double effectiveDegrees = MathUtil.clamp(
			degrees,
			Arm.MIN_ARM_ANGLE,
			Arm.MAX_ARM_ANGLE
		);
		
		this.setSetpoint(effectiveDegrees);
		this.enable();
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty("Arm Angle", this::getAngle, null);
		builder.addDoubleProperty("Left Encoder", this.leftEncoder::getPosition, null);
		builder.addDoubleProperty("Right Encoder", this.rightEncoder::getPosition, null);
		builder.addDoubleProperty("Left Encoder Zero Offset", this.leftEncoder::getZeroOffset, null);
		builder.addDoubleProperty("Right Encoder Zero Offset", this.rightEncoder::getZeroOffset, null);
		builder.addBooleanProperty("Left Upper Limit Switch", this.leftUpperLimitSwitch::get, null);
//		builder.addBooleanProperty("Right Upper Limit Switch", this.rightUpperLimitSwitch::get, null);
//		builder.addBooleanProperty("Left Lower Limit Switch", this.leftLowerLimitSwitch::get, null);
//		builder.addBooleanProperty("Right Lower Limit Switch", this.rightLowerLimitSwitch::get, null);
		builder.addBooleanProperty("Is left forward soft limit enabled", () -> this.leftMotorController.isSoftLimitEnabled(CANSparkBase.SoftLimitDirection.kForward), null);
		builder.addBooleanProperty("Is left reverse soft limit enabled", () -> this.leftMotorController.isSoftLimitEnabled(CANSparkBase.SoftLimitDirection.kReverse), null);
		builder.addBooleanProperty("Is right forward soft limit enabled", () -> this.rightMotorController.isSoftLimitEnabled(CANSparkBase.SoftLimitDirection.kForward), null);
		builder.addBooleanProperty("Is right reverse soft limit enabled", () -> this.rightMotorController.isSoftLimitEnabled(CANSparkBase.SoftLimitDirection.kReverse), null);
		builder.addDoubleProperty("P constant", this.getController()::getP, this.getController()::setP);
		builder.addDoubleProperty("D constant", this.getController()::getD, this.getController()::setD);
		
	}
	
	public class Commands {
		
		public Command raiseArm() {
			
			return Arm.this.startEnd(
				() -> Arm.this.rotate(true),
				Arm.this::stop
			).onlyWhile(Arm.this::isArmInsideLimits)
				.withName("Raise Arm");
			
		}
		
		public Command lowerArm() {
			
			return Arm.this.startEnd(
				() -> Arm.this.rotate(false),
				Arm.this::stop
			).onlyWhile(Arm.this::isArmInsideLimits).withName("Lower Arm");
			
		}
		
		public Command calibrateArm() {
			
			return Arm.this.runOnce(Arm.this::calibrateArm)
				.withName("Calibrate Arm")
				.ignoringDisable(true);
			
		}
		
		public Command rotateToAngle(double degrees) {
			
			return Arm.this.run(
				() -> Arm.this.rotateToAngle(degrees)
			).until(
				() -> Arm.this.getController().atSetpoint()
			);
			
		}
		
		public Command stop() {
			
			return Arm.this.runOnce(Arm.this::stop);
			
		}
		
	}
	
}
