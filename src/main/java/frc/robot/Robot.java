// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.configuration.Auton;
import frc.robot.configuration.DoublePreference;
import frc.robot.configuration.StartPosition;

import java.util.Optional;

public class Robot extends TimedRobot {
	
    private Command autonomousCommand;
    
    private RobotContainer robotContainer;
    
    @Override
    public void robotInit() {
        
        // Initialize preferences on the RoboRIO.
        DoublePreference.init();
        
        // Initialize the Shuffleboard starting position selector.
        StartPosition.initializeShuffleboardSelector();
        
        // Initialize the Shuffleboard auton selector.
        Auton.initializeShuffleboardSelector();
		
        // Initialize the RobotContainer.
        this.robotContainer = new RobotContainer();
        
        this.robotContainer.robotInit();
		
    }
    
    @Override
    public void autonomousInit() {
        
        Auton.runSelectedAuton(this.robotContainer);
        
    }
    
    @Override
    public void teleopInit() {
        
        // Stop any running autonomous command when teleop starts.
        if (this.autonomousCommand != null) this.autonomousCommand.cancel();
        
        // Start the robot's teleop mode.
        this.robotContainer.initTeleop();
        
    }
    
    @Override
    public void testInit() {
        
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        
    }
    
    @Override
    public void robotPeriodic() {
        
        this.robotContainer.robotPeriodic();
		
        // Start the command scheduler.
        CommandScheduler.getInstance().run();
		
    }
    
    @Override
    public void teleopPeriodic() {
        
        // Run the teleopPeriodic method of the RobotContainer.
        this.robotContainer.teleopPeriodic();
        
    }
    
    @Override
    public void teleopExit() {
        
        // Run the teleopExit method of the RobotContainer.
        this.robotContainer.teleopExit();
        
    }
}
