package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum StartPosition {
    
    MIDDLE(0),
    
    // on red, amp on right, on blue, amp on left
    AMP_SIDE(.35),
    
    // on red, source on left, on blue, source on right
    SOURCE_SIDE(-.35);
    
    public final double autonYSpeed;
    
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
