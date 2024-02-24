// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANDevice;

public class Shooter extends SubsystemBase {
	
	CANSparkMax leftShooterMotor, rightShooterMotor;
	
	public Shooter() {
        
        leftShooterMotor = new CANSparkMax(
            CANDevice.RIGHT_SHOOTER_MOTOR_CONTROLLER.id,
            MotorType.kBrushless
        );
        
        rightShooterMotor = new CANSparkMax(
            CANDevice.LEFT_SHOOTER_MOTOR_CONTROLLER.id,
            MotorType.kBrushless
        );
		
        rightShooterMotor.setInverted(false);
        leftShooterMotor.setInverted(true);
		
        leftShooterMotor.setIdleMode(IdleMode.kCoast);
        rightShooterMotor.setIdleMode(IdleMode.kCoast);
		
    }
    
    public void stop () {
		
        leftShooterMotor.stopMotor();
        rightShooterMotor.stopMotor();
		
    }
    
    public void runShooter (double leftMotorSpeed, double rightMotorSpeed) {
		
        leftShooterMotor.set(leftMotorSpeed);
        rightShooterMotor.set(rightMotorSpeed);
		
    }
        
    double leftSpeed = .5, rightSpeed = .5;
	
    public void runShooterTest () {
		
        leftShooterMotor.set(leftSpeed);
        rightShooterMotor.set(rightSpeed);
		
    }
    
    public void increaseShooterSpeed () {
		
        if (leftSpeed <= 1) leftSpeed += .05;
        if (rightSpeed <= 1) rightSpeed += .05;
		
    }
    
    public void decreaseShooterSpeed () {
		
        if (leftSpeed >= -1) leftSpeed -= .05;
        if (rightSpeed >= -1) rightSpeed -= .05;
		
    }
    
    @Override
    public void periodic() {
		
        // This method will be called once per scheduler run
		
    }
    
    @Override
    public void initSendable (SendableBuilder builder) {
		
        builder.addDoubleProperty("Left Motor Speed", () -> leftSpeed, null);
        builder.addDoubleProperty("Right Motor Speed", () -> rightSpeed, null);
		
    }
	
}
