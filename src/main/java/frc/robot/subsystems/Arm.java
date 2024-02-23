// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	
	CANSparkMax leftTuffbox, rightTuffbox;
	
	PIDController anglePID;
	
	SparkAbsoluteEncoder boreEncoder;
	
	public Arm(int leftTuffboxID, int rightTuffboxID) {
		
		leftTuffbox = new CANSparkMax(leftTuffboxID, MotorType.kBrushless);
		rightTuffbox = new CANSparkMax(rightTuffboxID, MotorType.kBrushless);
		
		// boreEncoder = rightTuffbox.getAbsoluteEncoder(Type.kDutyCycle);
		
		// leftTuffbox.setIdleMode(IdleMode.kBrake);
		// rightTuffbox.setIdleMode(IdleMode.kBrake);
		
		// Set the motor to be inverted so that it rotates in the same direction
		// as the other motor with the same input value.
		// rightTuffbox.setInverted(true);
		
		anglePID = new PIDController(.01, 0, 0);
		
	}
	
	public void stop() {
		
		leftTuffbox.stopMotor();
		rightTuffbox.stopMotor();
		
	}
	
	public double getAngleDegrees() {
		
		return boreEncoder.getPosition() - boreEncoder.getZeroOffset();
        
	}
	
	double rotationSpeed, motorSpeeds;
	
	public void setToAngle(double angleInDegrees) {
		
		rotationSpeed = anglePID.calculate(getAngleDegrees(), angleInDegrees);
		motorSpeeds = rotationSpeed / 2;
		
		leftTuffbox.set(motorSpeeds);
		rightTuffbox.set(motorSpeeds);
        
	}
	
	public void changeAngle(double speed) {
		
		leftTuffbox.set(speed);
		rightTuffbox.set(speed);
        
	}
	
	@Override
	public void periodic() {
        
		// This method will be called once per scheduler run
        
	}
	
}
