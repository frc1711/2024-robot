// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANDevice;
import frc.robot.constants.DoublePreference;

import java.util.stream.Stream;

public class Intake extends SubsystemBase {
    
    protected final CANSparkMax leftUpperMotorController;

    protected final CANSparkMax rightLowerMotorController;
    
    public final Intake.Commands commands;
    
    public Intake() {
        
        this.leftUpperMotorController = new CANSparkMax(
            CANDevice.LEFT_UPPER_INTAKE_MOTOR_CONTROLLER.id,
            MotorType.kBrushless
        );
        
        this.rightLowerMotorController = new CANSparkMax(
            CANDevice.RIGHT_LOWER_INTAKE_MOTOR_CONTROLLER.id,
            MotorType.kBrushless
        );
        
        this.rightLowerMotorController.setInverted(true);
        this.leftUpperMotorController.setInverted(false);
        
        this.getMotorControllerStream().forEach((motorController) -> {
            
            motorController.setIdleMode(IdleMode.kBrake);
            motorController.setOpenLoopRampRate(1);
            motorController.setSmartCurrentLimit(30);
            
        });
        
        this.commands = new Intake.Commands();
		
    }
    
    protected Stream<CANSparkMax> getMotorControllerStream() {
        
        return Stream.of(
            this.leftUpperMotorController,
            this.rightLowerMotorController
        );
        
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
    
    /**
     * Runs the intake in the normative direction at the standard speed.
     */
    public void intake() {
        
        this.intake(false);
        
    }
    
    /**
     * Runs the intake in the given direction at the standard speed.
     *
     * @param reversed Whether to run the intake in reverse.
     */
    public void intake(boolean reversed) {
        
        DoublePreference speedPreference = DoublePreference.INTAKE_SPEED;
        
        double speed = Preferences.getDouble(
            speedPreference.key,
            speedPreference.defaultValue
        );
        
        this.intake(speed, reversed);
        
    }
    
    /**
     * Runs the intake in the normative direction at the given relative speed.
     *
     * @param relativeSpeed The relative speed at which to run the intake (from
     * -1 to 1).
     */
    public void intake(double relativeSpeed) {
        
        this.intake(relativeSpeed, false);
        
    }
    
    /**
     * Runs the intake in the given direction at the given relative speed.
     *
     * @param relativeSpeed The relative speed at which to run the intake (from
     * -1 to 1).
     * @param reversed Whether to run the intake in reverse.
     */
    public void intake(double relativeSpeed, boolean reversed) {
        
        this.leftUpperMotorController.set(
            reversed ? -relativeSpeed : relativeSpeed
        );
        
        this.rightLowerMotorController.set(
            reversed ? -relativeSpeed : relativeSpeed
        );
        
    }
    
    /**
     * Returns true if the robot is currently holding a note.
     *
     * @return true if the robot is currently holding a note.
     */
    public boolean isHoldingNote() {
		
        // TODO - Implement this method.
        return false;
		
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        
        super.initSendable(builder);
        
        builder.addDoubleProperty(
            "Left Upper Motor Speed",
            this.leftUpperMotorController::get,
            null
        );
        
        builder.addDoubleProperty(
            "Right Lower Motor Speed",
            this.rightLowerMotorController::get,
            null
        );
        
    }
    
    public class Commands {
        
        public Command intake() {
            
            return Intake.this.startEnd(
                Intake.this::intake,
                Intake.this::stop
            );
            
        }
        
        public Command outtake() {
            
            return Intake.this.startEnd(
                () -> Intake.this.intake(true),
                Intake.this::stop
            );
            
        }
        
    }
	
}
