package frc.robot.controlsschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

@FunctionalInterface
public interface ControlsScheme {
	
	/**
	 * Configures the controls for this control scheme.
	 */
	void configureControls(
		RobotContainer robotContainer,
		CommandXboxController controller1,
		CommandXboxController controller2
	);
	
}
