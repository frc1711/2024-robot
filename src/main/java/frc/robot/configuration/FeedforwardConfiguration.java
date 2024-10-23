package frc.robot.configuration;

import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public enum FeedforwardConfiguration {

    FRONT_LEFT_DRIVE_MOTOR(
            Volts.of(0),

    );

    public final Measure<Voltage> kS;

    public final Measure<Per<Mult<Voltage, Time>, Distance>> kV;

    public final Measure<Per<Mult<Voltage, Mult<Time, Time>>, Distance>> kA;

    FeedforwardConfiguration(
        Measure<Voltage> kS,
        Measure<Per<Mult<Voltage, Time>, Distance>> kV,
        Measure<Per<Mult<Voltage, Mult<Time, Time>>, Distance>> kA
    ) {

        this.kS = kS;
        this.kV = kV;
        this.kA = kA;

    }

}
