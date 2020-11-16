package frc6831.util.geo2d;

import frc6831.util.Utl;
import org.jetbrains.annotations.NotNull;

/**
 * The description of a 2 dimensional vector represented by double values.
 */
public class Vector2d {
    static public final int VECTOR_ADD = 0;
    static public final int VECTOR_SUBTRACT = 1;

    private double m_i;
    private double m_j;

    /**
     * Instantiate a vector from a start location to an end location.
     *
     * @param xStart The X start location.
     * @param yStart The Y start location.
     * @param xEnd The X end location.
     * @param yEnd The Y end location.
     */
    public Vector2d(double xStart, double yStart, double xEnd, double yEnd) {
        m_i = xEnd - xStart;
        m_j = yEnd - yStart;
    }

    /**
     * Instantiate a vector given the X and Y lengths of the vector.
     *
     * @param dX
     * @param dY
     */
    public Vector2d(double dX, double dY) {
        m_i = dX;
        m_j = dY;
    }

    /**
     * Instantiate a vector that is the sum or difference of two vectors.
     *
     * @param v1
     * @param v2
     * @param vectorOp
     */
    public Vector2d(Vector2d v1, Vector2d v2, int vectorOp) {
        switch (vectorOp) {
            case VECTOR_ADD:
                m_i = v1.m_i + v2.m_i;
                m_j = v1.m_j + v2.m_j;
                break;
            case VECTOR_SUBTRACT:
                m_i = v2.m_i - v1.m_i;
                m_j = v2.m_j - v1.m_j;
                break;
        }
    }

    /**
     *
     * @return
     */
    public double getI() {
        return m_i;
    }

    /**
     *
     * @return
     */
    public double getJ() {
        return m_j;
    }

    /**
     *
     * @return
     */
    public Vector2d normalize() {
        double length = length();
        m_i /= length;
        m_j /= length;
        return this;
    }

    /**
     *
     * @return
     */
    public double length() {
        return Utl.length(m_i, m_j);
    }

    /**
     *
     * @param v
     * @return
     */
    public double dot(Vector2d v) {
        return (m_i * v.m_i) + (m_j * v.m_j);
    }

    /**
     *
     * @param scale
     * @return
     */
    public Vector2d scale(double scale) {
        m_i *= scale;
        m_j *= scale;
        return this;
    }

    /**
     * Negate, or reverse the direction of, the vector.
     *
     * @return The negated (reversed) vector.
     */
    @NotNull
    public Vector2d negate() {
        return scale(-1.0);
    }
}
