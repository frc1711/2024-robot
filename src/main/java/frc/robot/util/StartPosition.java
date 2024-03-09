package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * An enumeration representing the starting position of the robot on the field
 * at the beginning of the match.
 *
 * Also contains code relevant to initializing and using a Shuffleboard widget
 * for determining the starting position of the robot.
 */
public enum StartPosition {
    
    /**
     * A starting position in which the robot is positioned against the center
     * of the SUBWOOFER (the angled field element underneath the SPEAKER).
     */
    MIDDLE(0),
    
    /**
     * A starting position in which the robot is positioned against the side of
     * the SUBWOOFER (the angled field element underneath the SPEAKER) closest
     * to the AMP.
     *
     * On the red alliance, this entails the robot being positioned on the right
     * side of the SUBWOOFER (looking out from the perspective of the driver's
     * station). On the blue alliance, this entails the robot being positioned
     * on the left side of the SUBWOOFER.
     */
    AMP_SIDE(0.35),
    
    /**
     * A starting position in which the robot is positioned against the side of
     * the SUBWOOFER (the angled field element underneath the SPEAKER) closest
     * to the SOURCE.
     *
     * On the red alliance, this entails the robot being positioned on the left
     * side of the SUBWOOFER (looking out from the perspective of the driver's
     * station). On the blue alliance, this entails the robot being positioned
     * on the right side of the SUBWOOFER.
     */
    SOURCE_SIDE(-0.35);
    
    /**
     * The Shuffleboard widget used for selecting the starting position of the
     * robot.
     */
    private static final SendableChooser<StartPosition> SHUFFLEBOARD_SELECTOR =
        new SendableChooser<>();
    
    /**
     * The default start position to use if no start position is explicitly
     * selected.
     */
    private static final StartPosition DEFAULT_START_POSITION =
        StartPosition.MIDDLE;
    
    /**
     * A flag indicating whether or not the Shuffleboard starting position
     * selector has been initialized.
     */
    private static boolean hasShuffleboardSelectorBeenInitialized = false;
    
    /**
     * The relative speed at which the robot should move forward or backward
     * during autonomous mode, based on the selected starting position.
     */
    public final double autonYSpeed;
    
    /**
     * Constructs a new StartPosition with the given relative speed at which the
     * robot should move forward or backward during autonomous mode.
     *
     * @param autonYSpeed The relative speed at which the robot should move
     * forward or backward during autonomous mode.
     */
    StartPosition(double autonYSpeed) {
        
        double effectiveYSpeed = autonYSpeed;
        
        // Get our current alliance information from the FMS, otherwise
        // default to Red.
        Alliance alliance = DriverStation.getAlliance()
            .orElse(Alliance.Red);
        
        if (alliance == Alliance.Red) effectiveYSpeed *= -1;
        
        this.autonYSpeed = effectiveYSpeed;
        
    }
    
    /**
     * Initializes the Shuffleboard widget used for selecting the starting
     * position of the robot.
     */
    public static void initializeShuffleboardSelector() {
        
        if (StartPosition.hasShuffleboardSelectorBeenInitialized) return;
        else StartPosition.hasShuffleboardSelectorBeenInitialized = true;
        
        StartPosition.SHUFFLEBOARD_SELECTOR.setDefaultOption(
            StartPosition.DEFAULT_START_POSITION.getHumanReadableName(),
            StartPosition.DEFAULT_START_POSITION
        );
        
        for (StartPosition startPosition: StartPosition.values()) {
            
            if (startPosition == StartPosition.DEFAULT_START_POSITION) continue;
            
            StartPosition.SHUFFLEBOARD_SELECTOR.addOption(
                startPosition.getHumanReadableName(),
                startPosition
            );
            
        }
        
        Shuffleboard.getTab("Pre-match Tab").add(
            "Start Position Chooser",
            StartPosition.SHUFFLEBOARD_SELECTOR
        );
        
    }
    
    /**
     * Returns the start position selected by the driver from the Shuffleboard
     * widget.
     *
     * @return The start position selected by the driver from the Shuffleboard
     * widget.
     */
    public static StartPosition getSelectedStartPosition() {
        
        return StartPosition.SHUFFLEBOARD_SELECTOR.getSelected();
        
    }
    
    /**
     * Returns the human-readable name of this start position, having been
     * formatted to be more readable from the raw enum name.
     *
     * @return The human-readable name of this start position.
     */
    public String getHumanReadableName() {
        
        return StringUtilities.toTitleCase(
            this.name().replace('_', ' ')
        );
        
    }
    
}
