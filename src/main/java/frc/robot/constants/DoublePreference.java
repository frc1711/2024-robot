package frc.robot.constants;

import edu.wpi.first.wpilibj.Preferences;

public enum DoublePreference {
    
    FRONT_LEFT_SWERVE_MODULE_ENCODER_OFFSET("Front Left Swerve Module Encoder Offset", 0),
    
    FRONT_RIGHT_SWERVE_MODULE_ENCODER_OFFSET("Front Right Swerve Module Encoder Offset", 0),
    
    REAR_LEFT_SWERVE_MODULE_ENCODER_OFFSET("Rear Left Swerve Module Encoder Offset", 0),
    
    REAR_RIGHT_SWERVE_MODULE_ENCODER_OFFSET("Rear Right Swerve Module Encoder Offset", 0),

    SHOOTER_SPEED("Shooter Speed", 1),

    INTAKE_SPEED("Intake Speed", 0.5),

    ARM_SPEED("Arm Speed", 0.5),
    
    ARM_LEFT_ENCODER_ZERO_OFFSET("Arm Left Encoder Zero Offset", 0),
    
    ARM_RIGHT_ENCODER_ZERO_OFFSET("Arm Right Encoder Zero Offset", 0);

    public final String key;

    public final double defaultValue;

    DoublePreference(String key, double defaultValue) {

        this.key = key;
        this.defaultValue = defaultValue;

    }

    public static void init() {

        for (DoublePreference preference: DoublePreference.values()) {

            Preferences.initDouble(preference.key, preference.defaultValue);

        }

    }

}
