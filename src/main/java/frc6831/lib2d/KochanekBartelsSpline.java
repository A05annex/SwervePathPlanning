package frc6831.lib2d;

import org.jetbrains.annotations.NotNull;

import java.awt.geom.Point2D;
import java.lang.Iterable;
import java.util.Iterator;

public class KochanekBartelsSpline {

    static final double[][] m_basis = {
            {2.0, -2.0, 1.0, 1.0},
            {-3.0, 3.0, -2.0, -1.0},
            {0.0, 0.0, 1.0, 0.0},
            {1.0, 0.0, 0.0, 0.0}
    };

    private ControlPoint m_first = null;
    private ControlPoint m_last = null;

    public class PathPoint {
        public final double fieldX;
        public final double fieldY;
        public final double fieldHeading;

        public PathPoint(double fieldX, double fieldY, double fieldHeading) {
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.fieldHeading = fieldHeading;
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    public static class ControlPoint {
        public ControlPoint m_next = null;
        public ControlPoint m_last = null;
        public double m_fieldX = 0.0;
        public double m_fieldY = 0.0;
        public double m_fieldHeading = 0.0;
        public double m_time = 0.0;
        public boolean m_locationDerivativesEdited = false;
        public double m_dX;
        public double m_dY;
        public double m_dHeading;
        public double m_dHeadingOut;

        public ControlPoint() {
        }

        public void setFieldLocation(Point2D pt) {
            setFieldLocation(pt.getX(), pt.getY());
            // update the derivatives
        }

        public void setFieldLocation(double fieldX, double fieldY) {
            m_fieldX = fieldX;
            m_fieldY = fieldY;
            // update the derivatives
            updateLocationDerivatives();
            if (m_last != null) {
                m_last.updateLocationDerivatives();
            }
            if (m_next != null) {
                m_next.updateLocationDerivatives();
            }
        }

        public double getTangentX() {
            return m_fieldX + m_dX;
        }
        public double getTangentY() {
            return m_fieldY + m_dY;
        }

        public void setTangentLocation(Point2D pt) {
            setTangentLocation(pt.getX(), pt.getY());
            // update the derivatives
        }
        public void setTangentLocation(double fieldX, double fieldY) {
            m_dX = fieldX - m_fieldX;
            m_dY = fieldY - m_fieldY;
            m_locationDerivativesEdited = true;
        }

        private void updateLocationDerivatives() {
            if (!m_locationDerivativesEdited) {
                double fieldXprev = (m_last != null) ? m_last.m_fieldX : m_fieldX;
                double fieldYprev = (m_last != null) ? m_last.m_fieldY : m_fieldY;
                double fieldXnext = (m_next != null) ? m_next.m_fieldX : m_fieldX;
                double fieldYnext = (m_next != null) ? m_next.m_fieldY : m_fieldY;
                m_dX = 0.5 * (fieldXnext - fieldXprev);
                m_dY = 0.5 * (fieldYnext - fieldYprev);
            }
        }

        public void setFieldHeading(double heading) {
            m_fieldHeading = heading;
            // update the derivatives
            updateHeadingDerivative();
            if (m_last != null) {
                m_last.updateHeadingDerivative();
            }
            if (m_next != null) {
                m_next.updateHeadingDerivative();
            }
        }

        private void updateHeadingDerivative() {
            double fieldHeadingPrev = m_last != null ? m_last.m_fieldHeading : 0.0;
            double fieldHeadingNext = m_next != null ? m_next.m_fieldHeading : 0.0;
            m_dHeading = 0.5 * (fieldHeadingNext - fieldHeadingPrev);
        }

        /**
         * @param fieldX
         * @param fieldY
         * @param tolerance
         * @return Returns {@cade true} if the test point is over the control point, and {@code false} otherwise.
         */
        public boolean testOverControlPoint(double fieldX, double fieldY, double tolerance) {
            double dx = m_fieldX - fieldX;
            double dy = m_fieldY - fieldY;
            return Math.sqrt((dx * dx) + (dy * dy)) < tolerance;
        }

        /**
         * @param fieldX
         * @param fieldY
         * @param tolerance
         * @return Returns {@cade true} if the test point is over the tangent point, and {@code false} otherwise.
         */
        public boolean testOveTangentPoint(double fieldX, double fieldY, double tolerance) {
            double dx = (m_fieldX + m_dX) - fieldX;
            double dy = (m_fieldY + m_dY) - fieldY;
            return Math.sqrt((dx * dx) + (dy * dy)) < tolerance;
        }
    }

    // -----------------------------------------------------------------------------------------------------------------

    /**
     *
     */
    public class ControlPointIterator implements Iterator<ControlPoint>, Iterable<ControlPoint> {

        private ControlPoint m_current = m_first;

        public ControlPointIterator() {
        }


        @Override
        public boolean hasNext() {
            return m_current != null;
        }

        @Override
        public ControlPoint next() {
            ControlPoint current = m_current;
            m_current = current.m_next;
            return current;
        }

        @NotNull
        @Override
        public Iterator<ControlPoint> iterator() {
            return this;
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    public class PathIterator implements Iterator<PathPoint>, Iterable<PathPoint> {
        ControlPoint m_thisSegmentStart = m_first;
        ControlPoint m_thisSegmentEnd = m_first == null ? null : m_first.m_next;
        final double[][] m_segment = {
                {0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0}
        };
        double m_s = 0.0;
        double m_delta = 0.05;

        private PathIterator() {
            resetSegment();
            m_s = 0.0;
        }

        private void resetSegment() {
            if (null == m_thisSegmentEnd) {
                // we are done with this spline, just return.
                return;
            }
            m_s = m_delta;
            m_segment[0][0] = m_thisSegmentStart.m_fieldX;
            m_segment[1][0] = m_thisSegmentEnd.m_fieldX;
            m_segment[2][0] = m_thisSegmentStart.m_dX;
            m_segment[3][0] = m_thisSegmentEnd.m_dX;
            m_segment[0][1] = m_thisSegmentStart.m_fieldY;
            m_segment[1][1] = m_thisSegmentEnd.m_fieldY;
            m_segment[2][1] = m_thisSegmentStart.m_dY;
            m_segment[3][1] = m_thisSegmentEnd.m_dY;
            m_segment[0][2] = m_thisSegmentStart.m_fieldHeading;
            m_segment[1][2] = m_thisSegmentEnd.m_fieldHeading;
            m_segment[2][2] = m_thisSegmentStart.m_dHeadingOut;
            m_segment[3][2] = m_thisSegmentEnd.m_dHeading;
        }

        @Override
        public boolean hasNext() {
            return m_thisSegmentEnd != null;
        }

        @Override
        public PathPoint next() {
            // get the next point on the curve
            double[] s = {m_s * m_s * m_s, m_s * m_s, m_s, 1.0};
            double[] weights = {0.0, 0.0, 0.0, 0.0};
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    weights[i] += s[j] * m_basis[j][i];
                }
            }
            double[] field = {0.0, 0.0, 0.0};
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 4; j++) {
                    field[i] += weights[j] * m_segment[j][i];
                }
            }
            // get ready for the next
            m_s += m_delta;
            if (m_s > 1.0001) {
                // past the end of this segment for the next point, transition to the next segment
                m_thisSegmentStart = m_thisSegmentEnd;
                m_thisSegmentEnd = m_thisSegmentStart.m_next;
                resetSegment();
            }
            // create and return the path point
            return new PathPoint(field[0], field[1], field[2]);
        }

        @NotNull
        @Override
        public Iterator<PathPoint> iterator() {
            return this;
        }

    }

    // -----------------------------------------------------------------------------------------------------------------
    public KochanekBartelsSpline() {
    }

    public ControlPoint addControlPoint(Point2D pt) {
        return addControlPoint(pt.getX(), pt.getY(), 0.0);
    }

    public ControlPoint addControlPoint(double fieldX, double fieldY) {
        return addControlPoint(fieldX, fieldY, 0.0);
    }

    public ControlPoint addControlPoint(double fieldX, double fieldY, double fieldHeading) {
        ControlPoint newControlPoint = new ControlPoint();
        if (null == m_first) {
            m_first = newControlPoint;
        } else {
            m_last.m_next = newControlPoint;
            newControlPoint.m_last = m_last;
        }
        m_last = newControlPoint;
        newControlPoint.setFieldLocation(fieldX, fieldY);
        return newControlPoint;
    }

    public Iterable<ControlPoint> getControlPoints() {
        return new ControlPointIterator();
    }

    public Iterable<PathPoint> getCurveSegments() {
        return new PathIterator();
    }

}
