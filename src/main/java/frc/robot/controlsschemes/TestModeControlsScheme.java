package frc.robot.controlsschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.swerve.SwerveModule;

public class TestModeControlsScheme implements ControlsScheme {
	
	@Override
	public void configureControls(
		RobotContainer robotContainer,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		Arm arm = robotContainer.arm;
		
		controller1.leftBumper().whileTrue(arm.commands.lowerArm());
		controller1.rightBumper().whileTrue(arm.commands.raiseArm());
		
		controller1.x().whileTrue(
			arm.commands.runSysIdDynamicVoltageTest(
				SysIdRoutine.Direction.kForward
			)
		);
		
		controller1.y().whileTrue(
			arm.commands.runSysIdQuasistaticVoltageTest(
				SysIdRoutine.Direction.kForward
			)
		);
		
		controller1.a().whileTrue(
			arm.commands.runSysIdDynamicVoltageTest(
				SysIdRoutine.Direction.kReverse
			)
		);
		
		controller1.b().whileTrue(
			arm.commands.runSysIdQuasistaticVoltageTest(
				SysIdRoutine.Direction.kReverse
			)
		);
		
	}
	
}
