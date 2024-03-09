package frc.robot.controlsschemes;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SingleControllerTeleoperativeControlsScheme
	implements ControlsScheme {
	
	@Override
	public void configureControls(
		RobotContainer robot,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		(new ControlsSchemeBuilder(robot)).configureDefaultRobotCommands()
			.useControllerSelectButtonToCalibrateFieldRelativeHeading(controller1)
			.useControllerJoysticksForDriving(controller1)
			.useControllerDPadForSnappingToHeading(controller1)
			.useControllerTriggersForIntakingAndOuttaking(controller1)
			.useABXYButtonsForShooting(controller1);
		
	}
	
	@Override
	public void periodic(
		RobotContainer robot,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		(new ControlsSchemeBuilder(robot))
			.rumbleControllerWhenNoteInIntake(controller1);
		
	}
	
	@Override
	public void exit(
		RobotContainer robot,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		controller1.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
		
	}
}
