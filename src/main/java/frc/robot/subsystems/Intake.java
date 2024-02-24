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
        
        this.leftUpperMotorController = new CANSparkMax(
            CANDevice.LEFT_UPPER_INTAKE_MOTOR_CONTROLLER.id,
            MotorType.kBrushless
        );
        
        this.rightLowerMotorController = new CANSparkMax(
            CANDevice.RIGHT_LOWER_INTAKE_MOTOR_CONTROLLER.id,
            MotorType.kBrushless
        );
        
        this.rightLowerMotorController.setInverted(false);
        this.leftUpperMotorController.setInverted(true);
        
        this.rightLowerMotorController.setIdleMode(IdleMode.kBrake);
        this.leftUpperMotorController.setIdleMode(IdleMode.kBrake);
		
    }
    
    public void stop() {
        
        this.leftUpperMotorController.stopMotor();
        this.rightLowerMotorController.stopMotor();
		
    }
    
    double intakeMotorSpeed = .9;
    int speedMultiplier = 1;
    
    public void runIntake(boolean reverseMode) {
		
        if (reverseMode) this.speedMultiplier = 1;
        else this.speedMultiplier = -1;
        
        this.leftUpperMotorController.set(this.speedMultiplier * this.intakeMotorSpeed);
        this.rightLowerMotorController.set(this.speedMultiplier * this.intakeMotorSpeed);
		
    }
    
    public boolean isHoldingNote() {
		
        return false;
		
    }
	
}
