// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.DoublePreference;

public class Robot extends TimedRobot {
	
    private Command autonomousCommand;
    
    private RobotContainer robotContainer;
    
    @Override
    public void robotInit() {
        
        // Initialize preferences on the RoboRIO.
        DoublePreference.init();
		
        // Initialize the RobotContainer.
        robotContainer = new RobotContainer();
		
    }
    
    @Override
    public void robotPeriodic() {
		
        // Start the command scheduler.
        CommandScheduler.getInstance().run();
		
    }
    
    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
		
        autonomousCommand = robotContainer.getAutonomousCommand();
        
        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
			
            autonomousCommand.schedule();
			
        }
	
    }
    
    @Override
    public void teleopInit() {
		
        // Stop any running autonomous command when teleop starts.
        if (autonomousCommand != null) autonomousCommand.cancel();
        
        // Start the robot's teleop mode.
        robotContainer.initTeleop();
		
    }
    
    @Override
    public void testInit() {
		
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
		
    }
	
}
