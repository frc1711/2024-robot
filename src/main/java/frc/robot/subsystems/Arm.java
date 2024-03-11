// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.CANDevice;
import frc.robot.configuration.DoublePreference;
import frc.robot.util.ControlsUtilities;
import frc.robot.devicewrappers.RaptorsSparkAbsoluteEncoder;

import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;

public class Arm extends PIDSubsystem {
	
	protected static final Unit<Angle> ANGLE_UNITS = Degrees;
	
	protected final CANSparkMax leftMotorController;

	protected final CANSparkMax rightMotorController;
	
	protected final RaptorsSparkAbsoluteEncoder leftEncoder;
	
	protected final RaptorsSparkAbsoluteEncoder rightEncoder;
	
	protected Measure<Angle> restingAngle;
	
	public final Arm.Commands commands;
	
	public final Arm.Triggers triggers;
	
	public Arm() {
		
		super(new PIDController(
			DoublePreference.ARM_PID_KP.get(),
			0,
			DoublePreference.ARM_PID_KD.get()
		));
		
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
		
		this.leftEncoder = new RaptorsSparkAbsoluteEncoder(
			this.leftMotorController.getAbsoluteEncoder(
				SparkAbsoluteEncoder.Type.kDutyCycle
			),
			DoublePreference.ARM_LEFT_ENCODER_ZERO_OFFSET,
			Degrees,
			false
		);
		
		this.rightEncoder = new RaptorsSparkAbsoluteEncoder(
			this.rightMotorController.getAbsoluteEncoder(
				SparkAbsoluteEncoder.Type.kDutyCycle
			),
			DoublePreference.ARM_RIGHT_ENCODER_ZERO_OFFSET,
			Degrees,
			true
		);
		
		this.commands = new Arm.Commands();
		this.triggers = new Arm.Triggers();
		
		this.leftMotorController.setInverted(false);
		this.rightMotorController.setInverted(true);
		
		this.getMotorControllerStream().forEach((motorController) -> {
			
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
		
		PIDController armAnglePIDController = this.getController();
		
		armAnglePIDController.setTolerance(
			DoublePreference.ARM_PID_TOLERANCE_DEGREES.get()
		);
		
		// Set the natural resting angle of the arm.
		armAnglePIDController.setSetpoint(
			DoublePreference.ARM_MIN_ANGLE_DEGREES.get()
		);
		
		Shuffleboard.getTab("Subsystems").add(this.commands.calibrateArm());
		
	}
	
	@Override
	protected double getMeasurement() {
		
		return this.getAngle().in(Arm.ANGLE_UNITS);
		
	}
	
	@Override
	protected void useOutput(double output, double setpoint) {
		
		if (this.getController().atSetpoint()) {
			
			this.streamMotorControllers().forEach(
				(motorController) -> motorController.setVoltage(0)
			);
			return;
			
		}
		
		double currentAngle = this.getAngle().in(Arm.ANGLE_UNITS);
		boolean wouldOverrun = (
			(output > 0 && currentAngle > DoublePreference.ARM_MAX_ANGLE_DEGREES.get()) ||
			(output < 0 && currentAngle < DoublePreference.ARM_MIN_ANGLE_DEGREES.get())
		);
		double effectiveOutput = wouldOverrun ? 0 : output;
		
		this.getMotorControllerStream().forEach(
			(motorController) -> motorController.setVoltage(effectiveOutput)
		);
	
	}
	
	protected Stream<CANSparkMax> getMotorControllerStream() {

		return Stream.of(this.leftMotorController, this.rightMotorController);

	}

	protected Stream<RaptorsSparkAbsoluteEncoder> getEncoderStream() {

		return Stream.of(this.leftEncoder, this.rightEncoder);

	}
	
	/**
	 * Calibrates the arm by informing the encoders that their current angle is
	 * the angle at which the arm is calibrated.
	 *
	 * @see DoublePreference#ARM_CALIBRATION_ANGLE_DEGREES
	 */
	public void calibrateArm() {
		
		this.getEncoderStream().forEach(
			(encoder) -> encoder.calibrate(
				Degrees.of(DoublePreference.ARM_CALIBRATION_ANGLE_DEGREES.get())
			)
		);
		
	}
	
	public boolean isArmOutsideUpperLimit() {
		
		return this.getAngle().in(Degrees) >
			DoublePreference.ARM_MAX_ANGLE_DEGREES.get();
		
	}
	
	public boolean isArmOutsideLowerLimit() {
		
		return this.getAngle().in(Degrees) <
			DoublePreference.ARM_MIN_ANGLE_DEGREES.get();
		
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
		this.getMotorControllerStream().forEach(CANSparkMax::stopMotor);
		
	}
	
	public Measure<Angle> getAngle() {

		return BaseUnits.Angle.of(
			this.getEncoderStream()
				.mapToDouble((encoder) -> encoder.getPosition().baseUnitMagnitude())
				.average()
				.orElse(0)
		);
        
	}
	
	public void setRestingAngle(Measure<Angle> angle) {
		
		this.restingAngle = Degrees.of(ControlsUtilities.applyClamp(
			angle.in(Degrees),
			DoublePreference.ARM_MIN_ANGLE_DEGREES.get(),
			DoublePreference.ARM_MAX_ANGLE_DEGREES.get()
		));
		
	}
	
	public void rotate(boolean shouldRaise) {
		
		double speed = DoublePreference.ARM_SPEED.get();
		
		this.rotate(shouldRaise ? speed : -speed);
		
	}

	public void rotate(double speed) {
		
		this.getMotorControllerStream().forEach(
			(motorController) -> motorController.set(speed)
		);

	}
	
	public void rotateToAngle(Measure<Angle> angle) {
		
		double effectiveDegrees = ControlsUtilities.applyClamp(
			angle.in(Degrees),
			DoublePreference.ARM_MIN_ANGLE_DEGREES.get(),
			DoublePreference.ARM_MAX_ANGLE_DEGREES.get()
		);
		
		this.setSetpoint(effectiveDegrees);
		this.enable();
		
	}
	
	public boolean isArmAtAngle(Measure<Angle> angle, Measure<Angle> tolerance) {
		
		Measure<Angle> currentAngle = this.getAngle();
		Measure<Angle> minimumAngle = angle.minus(tolerance);
		Measure<Angle> maximumAngle = angle.plus(tolerance);
		
		return (
			currentAngle.gte(minimumAngle) &&
			currentAngle.lte(maximumAngle)
		);
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty(
			"Arm Angle (Degrees)",
			() -> this.getAngle().in(Degrees),
			null
		);
		
		builder.addDoubleProperty(
			"Arm Angle Setpoint (Degrees)",
			() -> this.getController().getSetpoint(),
			this.getController()::setSetpoint
		);
		
	}
	
	public class Commands {
		
		public Command raiseArm() {
			
			return Arm.this.startEnd(
				() -> Arm.this.rotate(true),
				Arm.this::stop
			).onlyWhile(() -> !Arm.this.isArmOutsideUpperLimit())
				.withName("Raise Arm");
			
		}
		
		public Command lowerArm() {
			
			return Arm.this.startEnd(
				() -> Arm.this.rotate(false),
				Arm.this::stop
			).onlyWhile(() -> !Arm.this.isArmOutsideLowerLimit())
				.withName("Lower Arm");
			
		}
		
		/**
		 * Returns a Command that moves the arm to the resting angle and does
		 * not finish until interrupted.
		 *
		 * @return A Command that moves the arm to the resting angle and does
		 * not finish until interrupted.
		 */
		public Command holdAtRestingAngle() {
			
			return Arm.this.startEnd(
				() -> Arm.this.rotateToAngle(Arm.this.restingAngle),
				() -> {}
			).handleInterrupt(Arm.this::stop);
			
		}
		
		/**
		 * Returns a Command that sets the resting angle of the arm and moves to
		 * the newly set resting angle, and remains active until interrupted.
		 *
		 * @param angle The angle to set as the new resting angle of the arm.
		 * @return A Command that sets the resting angle of the arm and moves to
		 * the newly set resting angle, and remains active until interrupted.
		 */
		public Command holdAtRestingAngle(Measure<Angle> angle) {
			
			return Arm.this.runOnce(
				() -> Arm.this.setRestingAngle(angle)
			).andThen(this.holdAtRestingAngle());
			
		}
		
		/**
		 * Returns a Command that moves the arm to the resting angle and
		 * finishes once the resting angle has been reached.
		 *
		 * @return A Command that moves the arm to the resting angle and
		 * finishes once the resting angle has been reached.
		 */
		public Command goToRestingAngle() {
			
			return this.holdAtRestingAngle()
				.until(Arm.this.triggers.armHasReachedSetpoint());
			
		}
		
		/**
		 * Returns a Command that sets the resting angle of the arm and moves to
		 * the newly set resting angle, finishing once the resting angle has
		 * been reached.
		 *
		 * @param angle The angle to set as the new resting angle of the arm.
		 * @return A Command that sets the resting angle of the arm and moves to
		 * the newly set resting angle, finishing once the resting angle has
		 * been reached.
		 */
		public Command goToRestingAngle(Measure<Angle> angle) {
			
			return Arm.this.runOnce(
				() -> Arm.this.setRestingAngle(angle)
			).andThen(this.goToRestingAngle());
			
		}
		
		/**
		 * Returns a Command that moves the arm to the specified angle and does
		 * not finish until interrupted.
		 *
		 * @param angle The angle to rotate the arm to.
		 * @return A Command that moves the arm to the specified angle and does
		 * not finish until interrupted.
		 */
		public Command holdAtAngle(Measure<Angle> angle) {
			
			return Arm.this.startEnd(
				() -> Arm.this.rotateToAngle(angle),
				() -> {}
			).handleInterrupt(Arm.this::stop);
			
		}
		
		/**
		 * Returns a Command that moves the arm to the specified angle and
		 * finishes once the specified angle has been reached.
		 *
		 * @param angle The angle to rotate the arm to.
		 * @return A Command that moves the arm to the specified angle and
		 * finishes once the specified angle has been reached.
		 */
		public Command goToAngle(Measure<Angle> angle) {
			
			return this.holdAtAngle(angle)
				.until(Arm.this.triggers.armHasReachedSetpoint());
			
		}
		
		public Command stop() {
			
			return Arm.this.runOnce(Arm.this::stop);
			
		}
		
		public Command calibrateArm() {
			
			return Arm.this.runOnce(Arm.this::calibrateArm)
				.withName("Calibrate Arm")
				.ignoringDisable(true);
			
		}
		
	}
	
	public class Triggers {
		
		public Trigger armHasReachedSetpoint() {
			
			return new Trigger(Arm.this.getController()::atSetpoint);
			
		}
		
		public Trigger armIsAtAngle(Measure<Angle> angle, Measure<Angle> tolerance) {
			
			return new Trigger(
				() -> Arm.this.isArmAtAngle(angle, tolerance)
			);
			
		}
		
	}
	
}
