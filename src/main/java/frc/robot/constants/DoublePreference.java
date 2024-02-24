package frc.robot.constants;

import edu.wpi.first.wpilibj.Preferences;

public enum DoublePreference {

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
