package frc.robot.controlsschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

import java.util.stream.Stream;

public class StandardTeleoperativeControlsScheme implements ControlsScheme {
	
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
			.useControllerTriggersForIntakingAndOuttaking(controller2)
			.useABXYButtonsForShooting(controller2);
		
	}
	
}
