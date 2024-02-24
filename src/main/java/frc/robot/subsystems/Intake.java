// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANDevice;

public class Intake extends SubsystemBase {
    
    protected final CANSparkMax leftUpperMotorController;

    protected final CANSparkMax rightLowerMotorController;
    
    public Intake() {
        
        leftUpperMotorController = new CANSparkMax(
            CANDevice.LEFT_UPPER_INTAKE_MOTOR_CONTROLLER.id,
            MotorType.kBrushless
        );
        
        rightLowerMotorController = new CANSparkMax(
            CANDevice.RIGHT_LOWER_INTAKE_MOTOR_CONTROLLER.id,
            MotorType.kBrushless
        );
        
        rightLowerMotorController.setInverted(false);
        leftUpperMotorController.setInverted(true);
        
        rightLowerMotorController.setIdleMode(IdleMode.kBrake);
        leftUpperMotorController.setIdleMode(IdleMode.kBrake);
		
    }
    
    public void stop () {
		
        leftUpperMotorController.stopMotor();
        rightLowerMotorController.stopMotor();
		
    }
    
    double intakeMotorSpeed = .9;
    int speedMultiplier = 1;
	
    public void runIntake (boolean reverseMode) {
		
        if (reverseMode) speedMultiplier = 1;
        else speedMultiplier = -1;
		
        leftUpperMotorController.set(speedMultiplier * intakeMotorSpeed);
        rightLowerMotorController.set(speedMultiplier * intakeMotorSpeed);
		
    }
    
    public void increaseIntakeSpeed () {
		
        if (intakeMotorSpeed <= 1) intakeMotorSpeed += .05;
		
    }
    
    public void decreaseIntakeSpeed () {
		
        if (intakeMotorSpeed >= -1) intakeMotorSpeed -= .05;
	
    }
    
    public boolean isHoldingNote () {
		
        return false;
		
    }
    
    @Override
    public void periodic() {
		
        // This method will be called once per scheduler run
		
    }
	
}
