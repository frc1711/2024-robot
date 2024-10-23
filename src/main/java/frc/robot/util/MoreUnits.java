package frc.robot.util;

import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public class MoreUnits {

    public static final Mult<Voltage, Time> VoltSeconds = Volts.mult(Second);

    public static final Per<Mult<Voltage, Time>, Angle> VoltSecondsPerRevolution = VoltSeconds.per(Revolution);

    public static final Mult<Voltage, Mult<Time, Time>> VoltSecondsSquared = Volts.mult(Seconds.mult(Seconds));

    public static final Per<Mult<Voltage, Mult<Time, Time>>, Angle> VoltSecondsSquaredPerRevolution = VoltSecondsSquared.per(Revolution);

}
