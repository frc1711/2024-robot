// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.CANDevice;
import frc.robot.configuration.DoublePreference;

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
            motorController.setSmartCurrentLimit(60);
            
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
        
        this.intake(DoublePreference.INTAKE_SPEED.get(), reversed);
        
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
    
    public class Commands {
        
        public Command intake() {
            
            return Intake.this.startEnd(
                Intake.this::intake,
                Intake.this::stop
            );
            
        }
        
        public Command intake(double speed) {
            
            return Intake.this.startEnd(
                () -> Intake.this.intake(speed),
                Intake.this::stop
            );
            
        }
        
        public Command featherIntake() {
            
            Command startIntaking = this.intake().withTimeout(2);
            Command featherIn = this.intake().withTimeout(0.5);
            Command featherOut = this.outtake().withTimeout(0.25);
            Command feather = featherIn.andThen(featherOut).repeatedly();
            
            return startIntaking.andThen(feather);
            
        }
        
        public Command outtake() {
            
            return Intake.this.startEnd(
                () -> Intake.this.intake(true),
                Intake.this::stop
            );
            
        }
        
        public Command outtake(double speed) {
            
            return Intake.this.startEnd(
                () -> Intake.this.intake(speed, true),
                Intake.this::stop
            );
            
        }
        
        public Command stop() {
            
            return Intake.this.runOnce(Intake.this::stop);
            
        }
        
    }
	
}
