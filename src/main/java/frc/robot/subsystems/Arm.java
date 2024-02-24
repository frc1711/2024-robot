// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.CANDevice;
import frc.robot.constants.DIODevice;
import frc.robot.constants.DoublePreference;

import java.util.stream.Stream;

public class Arm extends SubsystemBase {
	
	protected final CANSparkMax leftMotorController;

	protected final CANSparkMax rightMotorController;

	protected final DigitalInput leftUpperLimitSwitch;

	protected final SparkAbsoluteEncoder leftEncoder;

	protected final SparkAbsoluteEncoder rightEncoder;

//	protected final DigitalInput rightUpperLimitSwitch;

//	protected final DigitalInput leftLowerLimitSwitch;

//	protected final DigitalInput rightLowerLimitSwitch;

	protected final PIDController anglePID;
	
	public Arm() {

		this.leftMotorController = new CANSparkMax(
			CANDevice.LEFT_PIVOT_MOTOR_CONTROLLER.id,
			MotorType.kBrushless
		);

		this.rightMotorController = new CANSparkMax(
			CANDevice.RIGHT_PIVOT_MOTOR_CONTROLLER.id,
			MotorType.kBrushless
		);
		
		this.leftEncoder = leftMotorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
		this.rightEncoder = rightMotorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

		leftUpperLimitSwitch = new DigitalInput(DIODevice.LEFT_UPPER_ARM_LIMIT_SWITCH.id);
//		rightUpperLimitSwitch = new DigitalInput(DIODevice.RIGHT_UPPER_ARM_LIMIT_SWITCH.id);
//		leftLowerLimitSwitch = new DigitalInput(DIODevice.LEFT_LOWER_ARM_LIMIT_SWITCH.id);
//		rightLowerLimitSwitch = new DigitalInput(DIODevice.RIGHT_LOWER_ARM_LIMIT_SWITCH.id);
		
		this.leftMotorController.setInverted(false);
		this.leftEncoder.setInverted(false);
		this.rightMotorController.setInverted(true);
		this.rightEncoder.setInverted(true);
		
		this.streamMotorControllers().forEach((motorController) -> {
			
			motorController.setOpenLoopRampRate(0.5);
			motorController.setIdleMode(CANSparkBase.IdleMode.kBrake);
			
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
		
		this.anglePID = new PIDController(.01, 0, 0);
		
//		Shuffleboard.getTab("Subsystems").add(
//			this.calibrateArmCommand().withName("Reset Arm Encoder Zero Offsets").ignoringDisable(true)
//		);
		
		RobotContainer.putCommand("Reset Arm", new InstantCommand(this::calibrateArm, this), true);
		
	}

	protected Stream<CANSparkMax> streamMotorControllers() {

		return Stream.of(this.leftMotorController, this.rightMotorController);

	}

	protected Stream<SparkAbsoluteEncoder> streamEncoders() {

		return Stream.of(this.leftEncoder, this.rightEncoder);

	}
	
	/**
	 * Resets the 'zero angle' of the pivot encoders.
	 *
	 * Should be used when the arm is level with the chassis of the robot.
	 */
	public void calibrateArm() {
		
		System.out.println("Calibrating arm encoders...");
		
		this.leftEncoder.setZeroOffset(0);
		this.rightEncoder.setZeroOffset(0);
		
		double newLeftZeroOffset = this.leftEncoder.getPosition();
		double newRightZeroOffset = this.rightEncoder.getPosition();
		
		Preferences.setDouble(DoublePreference.ARM_LEFT_ENCODER_ZERO_OFFSET.key, newLeftZeroOffset);
		Preferences.setDouble(DoublePreference.ARM_RIGHT_ENCODER_ZERO_OFFSET.key, newRightZeroOffset);
		
		this.leftEncoder.setZeroOffset(newLeftZeroOffset);
		this.rightEncoder.setZeroOffset(newRightZeroOffset);
		
	}
	
	public Command calibrateArmCommand() {
		
		return this.runOnce(this::calibrateArm);
		
	}

	protected boolean areUpperLimitsTripped() {

		return this.leftUpperLimitSwitch.get()/* || this.rightUpperLimitSwitch.get()*/;

	}

	protected boolean areLowerLimitsTripped() {

		// return this.leftLowerLimitSwitch.get() || this.rightLowerLimitSwitch.get();

		return false;

	}
	
	public void stop() {

		this.streamMotorControllers().forEach(CANSparkMax::stopMotor);
		
	}
	
	public double getAngleDegrees() {

		return this.streamEncoders()
			.mapToDouble(SparkAbsoluteEncoder::getPosition)
			.average()
			.orElse(0);
        
	}
	
//	public void setToAngle(double angleInDegrees) {
//
//		rotationSpeed = anglePID.calculate(getAngleDegrees(), angleInDegrees);
//		motorSpeeds = rotationSpeed / 2;
//
//		leftMotorController.set(motorSpeeds);
//		rightMotorController.set(motorSpeeds);
//
//	}
	
	public void rotate(boolean shouldRaise) {
		
		DoublePreference preference = DoublePreference.ARM_SPEED;
		double speed = Preferences.getDouble(preference.key, preference.defaultValue);
		
		this.rotate(shouldRaise ? speed : -speed);
		
	}

	public void rotate(double speed) {

		boolean wouldOverrunLimits = (
			(speed > 0 && this.areUpperLimitsTripped()) ||
			(speed < 0 && this.areLowerLimitsTripped())
		);

		double effectiveSpeed = wouldOverrunLimits ? 0 : speed;

		this.streamMotorControllers().forEach((motorController) -> motorController.set(effectiveSpeed));

	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty("Left Encoder", this.leftEncoder::getPosition, null);
		builder.addDoubleProperty("Right Encoder", this.rightEncoder::getPosition, null);
		builder.addBooleanProperty("Left Upper Limit Switch", this.leftUpperLimitSwitch::get, null);
//		builder.addBooleanProperty("Right Upper Limit Switch", this.rightUpperLimitSwitch::get, null);
//		builder.addBooleanProperty("Left Lower Limit Switch", this.leftLowerLimitSwitch::get, null);
//		builder.addBooleanProperty("Right Lower Limit Switch", this.rightLowerLimitSwitch::get, null);
		builder.addBooleanProperty("Is left forward soft limit enabled", () -> this.leftMotorController.isSoftLimitEnabled(CANSparkBase.SoftLimitDirection.kForward), null);
		builder.addBooleanProperty("Is left reverse soft limit enabled", () -> this.leftMotorController.isSoftLimitEnabled(CANSparkBase.SoftLimitDirection.kReverse), null);
		builder.addBooleanProperty("Is right forward soft limit enabled", () -> this.rightMotorController.isSoftLimitEnabled(CANSparkBase.SoftLimitDirection.kForward), null);
		builder.addBooleanProperty("Is right reverse soft limit enabled", () -> this.rightMotorController.isSoftLimitEnabled(CANSparkBase.SoftLimitDirection.kReverse), null);
		
	}
	
}
