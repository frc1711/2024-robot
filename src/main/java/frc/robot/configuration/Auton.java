package frc.robot.configuration;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auton.Fellowship;
import frc.robot.commands.auton.King;
import frc.robot.commands.auton.TwoTowers;
import frc.robot.commands.auton.framework.basic.SwerveAuton;

import java.util.function.Function;

public enum Auton {
	
	NONE(
		"None (No Auton)",
		robot -> new WaitCommand(0)
	),
	
	ONE_NOTE(
		"Fellowship of the Ring (One Note)",
			Fellowship::new
	),
	
	TWO_NOTE(
		"The Two Towers (Two Note)",
		TwoTowers::new
	),
	
	THREE_NOTE(
		"Return of the King (Three Note)",
		King::new
	),
	
	DISTANCE_CONFIG(
		"An Unexpected Journey (Distance Config)",
		robot -> new SwerveAuton(
			robot,
			DoublePreference.DISTANCE_CONFIG_AUTON_X_SPEED.get(),
			DoublePreference.DISTANCE_CONFIG_AUTON_Y_SPEED.get(),
			robot.swerve.getFieldRelativeHeadingRotation2d()
		).raceWith(
			new WaitCommand(DoublePreference.DISTANCE_CONFIG_AUTON_TIME.get())
		)
	);
	
	/**
	 * The Shuffleboard widget used for selecting the auton to run.
	 */
	private static final SendableChooser<Auton> SHUFFLEBOARD_SELECTOR =
		new SendableChooser<>();
	
	/**
	 * The default auton to run if no auton is explicitly selected.
	 */
	private static final Auton DEFAULT_AUTON = Auton.TWO_NOTE;
	
	/**
	 * A flag indicating whether or not the Shuffleboard auton selector has been
	 * initialized.
	 */
	private static boolean hasShuffleboardSelectorBeenInitialized = false;
	
	/**
	 * The human-readable name of this auton.
	 */
	private final String humanReadableName;
	
	/**
	 * The function that supplies the Command object for this auton.
	 */
	private final Function<RobotContainer, Command> commandSupplier;
	
	/**
	 * Constructs a new Auton with the given name and command supplier.
	 *
	 * @param name The human-readable name of this auton.
	 * @param commandFunction The function that supplies the Command object for
	 * this auton.
	 */
	Auton(String name, Function<RobotContainer, Command> commandFunction) {
		
		this.humanReadableName = name;
		this.commandSupplier = commandFunction;
		
	}
	
	/**
	 * Initializes the Shuffleboard widget used for selecting the starting
	 * position of the robot.
	 */
	public static void initializeShuffleboardSelector() {
		
		if (Auton.hasShuffleboardSelectorBeenInitialized) return;
		else Auton.hasShuffleboardSelectorBeenInitialized = true;
		
		Auton.SHUFFLEBOARD_SELECTOR.setDefaultOption(
			Auton.DEFAULT_AUTON.getHumanReadableName(),
			Auton.DEFAULT_AUTON
		);
		
		for (Auton auton: Auton.values()) {
			
			if (auton == Auton.DEFAULT_AUTON) continue;
			
			Auton.SHUFFLEBOARD_SELECTOR.addOption(
				auton.getHumanReadableName(),
				auton
			);
			
		}
		
		Shuffleboard.getTab("Pre-match Tab").add(
			"Auton Chooser",
			Auton.SHUFFLEBOARD_SELECTOR
		);
		
	}
	
	/**
	 * Returns the auton selected by the driver from the Shuffleboard widget.
	 *
	 * @return The auton selected by the driver from the Shuffleboard widget.
	 */
	public static Auton getSelectedAuton() {
		
		return Auton.SHUFFLEBOARD_SELECTOR.getSelected();
		
	}
	
	/**
	 * Runs the auton currently selected by the Shuffleboard widget.
	 *
	 * @param robot The robot on which to run the auton.
	 */
	public static void runSelectedAuton(RobotContainer robot) {
		
		Auton.getSelectedAuton().getCommand(robot).schedule();
		
	}
	
	/**
	 * Returns the Command object for this auton.
	 *
	 * @param robot The robot on which to run the auton.
	 * @return The Command object for this auton.
	 */
	public Command getCommand(RobotContainer robot) {
		
		return this.commandSupplier.apply(robot);
		
	}
	
	/**
	 * Returns the human-readable name of this auton.
	 *
	 * @return The human-readable name of this auton.
	 */
	public String getHumanReadableName() {
		
		return this.humanReadableName;
		
	}
	
}
