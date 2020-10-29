package frc6831.util;

/**
 * This is a utility class with some commonly used math constants and utility methods
 */
public final class Utl {

    public static final double PI = Math.PI;
    public static final double NEG_PI = -Math.PI;
    public static final double TWO_PI = Math.PI * 2.0;
    public static final double PI_OVER_2 = Math.PI * 0.5;
    public static final double NEG_PI_OVER_2 = -(Math.PI * 0.5);

    /**
     * This class is all static constants and methods, it cannot be instantiated.
     */
    private Utl() {}

    /**
     * Get the length of an n-dimensional set of lengths.
     *
     * @param values The dimensional lengths. The number of dimensional lengths is variable and may be zero.
     * @return The n-dimensional length, 0.0 of not dimensional lengths are specified.
     */
    public static double length(double ... values) {
        double lengthSquared = 0.0;
        for (double v : values) {
            lengthSquared += v * v;
        }
        return Math.sqrt(lengthSquared);
    }

    /**
     * Get the maximum value of an arbitrary number of values.
     *
     * @param values The values. The number of values is variable and may be zero.
     * @return The maximum value, {@link Double#NEGATIVE_INFINITY} if no values are specified.
     */
    public static double max(double ... values) {
        double max = Double.NEGATIVE_INFINITY;
        for (double v : values) {
            if (v > max) {
                max = v;
            }
        }
        return max;
    }

    /**
     * Get the minimum value of an arbitrary number of values.
     *
     * @param values The values. The number of values is variable and may be zero.
     * @return The minimum value, {@link Double#POSITIVE_INFINITY} if no values are specified.
     */
    public static double min(double ... values) {
        double min = Double.POSITIVE_INFINITY;
        for (double v : values) {
            if (v < min) {
                min = v;
            }
        }
        return min;
    }

}
