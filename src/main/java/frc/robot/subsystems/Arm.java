// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANDevice;

public class Arm extends SubsystemBase {
	
	CANSparkMax leftToughbox, rightToughbox;
	
	PIDController anglePID;
	
	SparkAbsoluteEncoder boreEncoder;
	
	public Arm() {
		
		leftToughbox = new CANSparkMax(
			CANDevice.LEFT_PIVOT_MOTOR_CONTROLLER.id,
			MotorType.kBrushless
		);
		
		rightToughbox = new CANSparkMax(
			CANDevice.RIGHT_PIVOT_MOTOR_CONTROLLER.id,
			MotorType.kBrushless
		);
		
		// boreEncoder = rightToughbox.getAbsoluteEncoder(Type.kDutyCycle);
		
		// leftToughbox.setIdleMode(IdleMode.kBrake);
		// rightToughbox.setIdleMode(IdleMode.kBrake);
		
		// Set the motor to be inverted so that it rotates in the same direction
		// as the other motor with the same input value.
		// rightToughbox.setInverted(true);
		
		anglePID = new PIDController(.01, 0, 0);
		
	}
	
	public void stop() {
		
		leftToughbox.stopMotor();
		rightToughbox.stopMotor();
		
	}
	
	public double getAngleDegrees() {
		
		return boreEncoder.getPosition() - boreEncoder.getZeroOffset();
        
	}
	
	double rotationSpeed, motorSpeeds;
	
	public void setToAngle(double angleInDegrees) {
		
		rotationSpeed = anglePID.calculate(getAngleDegrees(), angleInDegrees);
		motorSpeeds = rotationSpeed / 2;
		
		leftToughbox.set(motorSpeeds);
		rightToughbox.set(motorSpeeds);
        
	}
	
	public void changeAngle(double speed) {
		
		leftToughbox.set(speed);
		rightToughbox.set(speed);
        
	}
	
	@Override
	public void periodic() {
        
		// This method will be called once per scheduler run
        
	}
	
}
