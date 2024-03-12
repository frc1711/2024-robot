package frc.robot.configuration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ComplexCommands;
import frc.robot.RobotContainer;
import frc.robot.commands.auton.King;
import frc.robot.commands.auton.TwoTowers;
import frc.robot.commands.auton.framework.basic.SwerveAuton;

import java.util.function.Function;

import static edu.wpi.first.units.Units.Degrees;

public enum Auton {
	
	NONE(
		"None (No Auton)",
		robot -> new WaitCommand(0)
	),
	
	CROSS_THE_LINE(
		"Battle of the Five Armies (No Note - Cross the Line)",
		robot -> new SwerveAuton(
			robot,
			0.35,
			0,
			robot.swerve.getFieldRelativeHeadingRotation2d()
		).withTimeout(2)
	),
	
	ONE_NOTE(
		"Fellowship of the Ring (One Note)",
		robot -> {
			
			double xSpeed = 0.35;
			double ySpeed = 0;
			
			StartPosition startPosition = StartPosition.getSelectedStartPosition();
			Alliance alliance = DriverStation.getAlliance()
				.orElse(Auton.getDefaultAlliance());
			
			if (
				(startPosition == StartPosition.SOURCE_SIDE && alliance == Alliance.Red) ||
				(startPosition == StartPosition.AMP_SIDE && alliance == Alliance.Blue)
			) ySpeed = -0.35;
			else if (
				(startPosition == StartPosition.AMP_SIDE && alliance == Alliance.Red) ||
				(startPosition == StartPosition.SOURCE_SIDE && alliance == Alliance.Blue)
			) ySpeed = 0.35;
			
			return new WaitCommand(DoublePreference.AUTON_START_DELAY.get())
				.andThen(ComplexCommands.shootAtAngle(
					robot,
					Degrees.of(55),
					1
				).withTimeout(2))
				.andThen(new WaitCommand(DoublePreference.AUTON_ROLLOUT_DELAY.get()))
				.andThen(new SwerveAuton(
					robot,
					xSpeed,
					ySpeed,
					robot.swerve.getFieldRelativeHeadingRotation2d()
				).withTimeout(1.15));
			
		}
	),
	
	TWO_NOTE(
		"The Two Towers (Two Note)",
		TwoTowers::new
	),
	
	THREE_NOTE(
		"Return of the King (Three Note)",
		King::new
	);
	
	/**
	 * The Shuffleboard widget used for selecting the auton to run.
	 */
	private static final SendableChooser<Auton> SHUFFLEBOARD_SELECTOR =
		new SendableChooser<>();
	
	/**
	 * The default Alliance color to assume if no alliance color is able to be
	 * fetched from the FMS.
	 */
	private static final Alliance DEFAULT_AUTON_ALLIANCE = Alliance.Red;
	
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
	
	public static Alliance getDefaultAlliance() {
		
		return Auton.DEFAULT_AUTON_ALLIANCE;
		
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
