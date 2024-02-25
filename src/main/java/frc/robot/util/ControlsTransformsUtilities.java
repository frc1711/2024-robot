package frc.robot.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ControlsTransformsUtilities {

    public static DoubleSupplier applyDeadband(DoubleSupplier input, double deadband) {

        return () -> ControlsUtilities.applyDeadband(input.getAsDouble(), deadband);

    }

    public static Supplier<Point> applyCircularDeadband(Supplier<Point> input, double deadband) {

        return () -> {

            Point point = input.get();

            double hypotenuse = Math.sqrt(Math.pow(point.x, 2) + Math.pow(point.y, 2));

            if (Math.abs(hypotenuse) >= deadband) return point;
            else return new Point(0, 0);

        };

    }

}
