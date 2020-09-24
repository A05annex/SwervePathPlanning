package frc6831.lib2d;

import org.jetbrains.annotations.NotNull;

import java.awt.geom.Point2D;
import java.lang.Iterable;
import java.util.Iterator;

/**
 * This is an implementation of the
 * <a href="https://en.wikipedia.org/wiki/Kochanek%E2%80%93Bartels_spline">Kochanek-Bartels Spline</a> designed
 * for interactive editing of the tangent vector to implicitly control bias and tension. There is no continuity
 * control.
 *
 * When control points are created the tangent (derivatives) at that control point and surrounding control points are
 * computed using the <a href="https://en.wikipedia.org/wiki/Cubic_Hermite_spline">Cardinal-Spline</a> formulation
 * with the default tension specified by {@link #DEFAULT_TENSION}. The tangent is adjusted using a control handle
 * which intuitively manipulates the shape of the spline at the control point to implicitly edit tension and bias.
 *
 * This class is primarily a container-editor for a doubly-linked list of control points, and a factory for
 * path point iterators (iterating through the path points), or path point followers (generating a series of
 * path points at specified times along the path).
 */
public class KochanekBartelsSpline {

    /**
     * The length of the heading control handle in meters.
     */
    private static final double ROBOT_HEADING_HANDLE = 1.0;
    /**
     * In the formulation of the spline the tension scales the derivative. This was a tension selected for best
     * default appearance of the spline i.e. the default spline best represents the intent of the path planner.
     */
    private static final double DEFAULT_TENSION = 0.7;
    /**
     * A scale factor applied to the derivative when computing the position of the editing handle.
     */
    private static final double DERIVATIVE_UI_SCALE = 0.5;

    /**
     * The basis matrix that provides the weighting of the [s] matrix (position on the segment of the spline to
     * various powers) as applied to the start and end positions and derivatives.
     */
    static final double[][] m_basis = {
            {2.0, -2.0, 1.0, 1.0},
            {-3.0, 3.0, -2.0, -1.0},
            {0.0, 0.0, 1.0, 0.0},
            {1.0, 0.0, 0.0, 0.0}
    };

    /**
     * The first control point in this doubly-linked list of control points for the spline.
     */
    private ControlPoint m_first = null;
    /**
     * The last control point in this doubly-linked list of control points for the spline.
     */
    private ControlPoint m_last = null;

    // -----------------------------------------------------------------------------------------------------------------
    // PathPoint - a generated point on the path that includes expected field position and heading as well as
    // forward speed,/ strafe speed, and rotation speed for the robot.
    // -----------------------------------------------------------------------------------------------------------------

    /**
     * The representation of a generated point along the path.
     */
    public static class PathPoint {
        /**
         * The point on the field where the robot should be when it reaches this point in the path.
         */
        public final Point2D.Double fieldPt;
        /**
         * The field heading for the robot when it reaches this point on the path
         */
        public final double fieldHeading;
        /**
         * The forward chassis velocity of the robot in meters/sec.
         */
        public final double speedForward;
        /**
         * The strafe chassis velocity of the robot in meters/sec.
         */
        public final double speedStrafe;
        /**
         * The rotation speed of the robot in radians/sec.
         */
        public final double speedRotation;

        /**
         * Instantiate a Path Point.
         * @param fieldX The expected field X position of the robot in meters.
         * @param fieldY The expected field Y position of the robot in meters.
         * @param fieldHeading The expected heading of the robot in radians.
         * @param speedForward The forward chassis speed of the robot in meters/sec.
         * @param speedStrafe The strafe chassis velocity of the robot in meters/sec.
         * @param speedRotation The rotation speed of the robot in radians/sec.
         */
        public PathPoint(double fieldX, double fieldY, double fieldHeading,
                         double speedForward, double speedStrafe, double speedRotation) {
            this.fieldPt = new Point2D.Double(fieldX, fieldY);
            this.fieldHeading = fieldHeading;
            this.speedForward = speedForward;
            this.speedStrafe = speedStrafe;
            this.speedRotation = speedRotation;
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // ControlPoint - a control point with heading and derivatives for the spline
    // -----------------------------------------------------------------------------------------------------------------

    /**
     * This class represents a control point and has the operations that manipulate a control point in context with
     * its surrounding control points. Note that control points are maintained as a doubly-linked list because
     * manipulation of a control point affects derivatives of the adjacent control points.
     */
    public static class ControlPoint {
        public ControlPoint m_next = null;
        public ControlPoint m_last = null;
        public double m_fieldX = 0.0;
        public double m_fieldY = 0.0;
        public double m_fieldHeading = Math.PI/4.0;
        public double m_time = 0.0;
        public boolean m_locationDerivativesEdited = false;
        public double m_dX;
        public double m_dY;
        public double m_dHeading;

        /**
         * Instantiate a {@link ControlPoint}.
         */
        public ControlPoint() {
        }

        /**
         * Instantiate a control point and set the time this control point should be reached when the path
         * is traversed.
         *
         * @param timeInSec (double) The time this control point should be reached (in seconds).
         */
        public ControlPoint(double timeInSec) {
            m_time = timeInSec;
        }

        /**
         *
         *
         * @param pt
         */
        public void setFieldLocation(Point2D pt) {
            setFieldLocation(pt.getX(), pt.getY());
            // update the derivatives
        }

        /**
         *
         * @param fieldX
         * @param fieldY
         */
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

        /**
         *
         */
        private void updateLocationDerivatives() {
            // NOTE: If the derivative has been edited, then we assume the edited derivative is the intended
            // derivative and should not be recomputed when the control point is moved.
            if (!m_locationDerivativesEdited) {
                double fieldXprev = (m_last != null) ? m_last.m_fieldX : m_fieldX;
                double fieldYprev = (m_last != null) ? m_last.m_fieldY : m_fieldY;
                double fieldXnext = (m_next != null) ? m_next.m_fieldX : m_fieldX;
                double fieldYnext = (m_next != null) ? m_next.m_fieldY : m_fieldY;
                m_dX = DEFAULT_TENSION * (fieldXnext - fieldXprev);
                m_dY = DEFAULT_TENSION * (fieldYnext - fieldYprev);
            }
        }

        /**
         *
         * @return
         */
        public double getTangentX() {
            return m_fieldX + (DERIVATIVE_UI_SCALE * m_dX);
        }

        /**
         *
         * @return
         */
        public double getTangentY() {
            return m_fieldY + (DERIVATIVE_UI_SCALE * m_dY);
        }

        /**
         *
         * @param pt
         */
        public void setTangentLocation(Point2D pt) {
            setTangentLocation(pt.getX(), pt.getY());
        }

        /**
         *
         * @param fieldX
         * @param fieldY
         */
        public void setTangentLocation(double fieldX, double fieldY) {
            m_dX = (fieldX - m_fieldX) / DERIVATIVE_UI_SCALE;
            m_dY = (fieldY - m_fieldY) / DERIVATIVE_UI_SCALE;
            m_locationDerivativesEdited = true;
        }

        /**
         *
         * @return
         */
        public double getHeadingX() {
            return m_fieldX + (ROBOT_HEADING_HANDLE * Math.sin(m_fieldHeading));
        }

        /**
         *
         * @return
         */
        public double getHeadingY() {
            return m_fieldY + (ROBOT_HEADING_HANDLE * Math.cos(m_fieldHeading));
        }

        /**
         *
         * @param pt
         */
        public void setHeadingLocation(Point2D pt) {
            // OK, the simple action here is to look at the current mouse position relative to the control
            // point position, use the atan2, and get a heading. However, tis does not handle the -180/180 degree
            // transition, so we need some logic like the NavX logic. for passing over the boundary
            setFieldHeading(Math.atan2(pt.getX() - m_fieldX, pt.getY() - m_fieldY));
        }

        /**
         *
         * @param heading
         */
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

        /**
         *
         */
        private void updateHeadingDerivative() {
            double fieldHeadingPrev = m_last != null ? m_last.m_fieldHeading : m_fieldHeading;
            double fieldHeadingNext = m_next != null ? m_next.m_fieldHeading : m_fieldHeading;
            m_dHeading = DEFAULT_TENSION * (fieldHeadingNext - fieldHeadingPrev);
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
            double dx = getTangentX() - fieldX;
            double dy = getTangentY() - fieldY;
            return Math.sqrt((dx * dx) + (dy * dy)) < tolerance;
        }

        /**
         * @param fieldX
         * @param fieldY
         * @param tolerance
         * @return Returns {@cade true} if the test point is over the heading point, and {@code false} otherwise.
         */
        public boolean testOveHeadingPoint(double fieldX, double fieldY, double tolerance) {
            double dx = getHeadingX() - fieldX;
            double dy = getHeadingY() - fieldY;
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
    private abstract class PathGenerator {
        /**
         * The start of the segment being generated
         */
        ControlPoint m_thisSegmentStart = m_first;
        /**
         * The end of the segment being generated.
         */
        ControlPoint m_thisSegmentEnd = m_first == null ? null : m_first.m_next;
        /**
         * The basis matrix.
         */
        final double[][] m_segment = {
                {0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0}
        };

        protected void resetSegment() {
            if (null == m_thisSegmentEnd) {
                // we are done with this spline, just return.
                return;
            }
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
            m_segment[2][2] = m_thisSegmentStart.m_dHeading;
            m_segment[3][2] = m_thisSegmentEnd.m_dHeading;
        }

        public PathPoint getPointOnSegment(double sValue) {
            // get the next point on the curve
            double[] s = {sValue * sValue * sValue, sValue * sValue, sValue, 1.0};
            double[] ds = {3.0 * sValue * sValue, 2.0 * sValue, 1.0, 0.0};
            double[] weights = {0.0, 0.0, 0.0, 0.0};
            double[] dWeights = {0.0, 0.0, 0.0, 0.0};
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    weights[i] += s[j] * m_basis[j][i];
                    dWeights[i] += ds[j] * m_basis[j][i];
                }
            }
            double[] field = {0.0, 0.0, 0.0};
            double[] dField = {0.0, 0.0, 0.0};
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 4; j++) {
                    field[i] += weights[j] * m_segment[j][i];
                    dField[i] += dWeights[j] * m_segment[j][i];
                }
            }
            // OK, the position derivatives are X and Y relative to the field. These need to be transformed to
            // robot relative forward and strafe.
            double sinHeading = Math.sin(field[2]);
            double cosHeading = Math.cos(field[2]);
            double forward = (dField[0] * sinHeading) + (dField[1] * cosHeading);
            double strafe = (dField[0] * cosHeading) - (dField[1] * sinHeading);
            // create and return the path point
            return new PathPoint(field[0], field[1], field[2], forward, strafe, dField[2]);
        }
    }

    /**
     * An iterator for points along a path.
     */
    public class PathIterator extends PathGenerator implements Iterator<PathPoint>, Iterable<PathPoint> {
        /**
         * The current position on the segment being generated, from 0.0 being on {@link #m_thisSegmentStart}
         * to 1.0 being on {@link #m_thisSegmentEnd}.
         */
        double m_s = 0.0;
        /**
         * The point spacing increment on the curve.
         */
        final double m_delta;

        private PathIterator(double delta) {
            resetSegment();
            m_s = 0.0;
            m_delta = delta;
        }


        @Override
        public boolean hasNext() {
            return m_thisSegmentEnd != null;
        }

        @Override
        public PathPoint next() {
            // get the next point on the curve
            PathPoint pathPoint = getPointOnSegment(m_s);
            // get ready for the next
            m_s += m_delta;
            if (m_s > 1.0001) {
                // past the end of this segment for the next point, transition to the next segment
                m_thisSegmentStart = m_thisSegmentEnd;
                m_thisSegmentEnd = m_thisSegmentStart.m_next;
                m_s = m_delta;
                resetSegment();
            }
            // create and return the path point
            return pathPoint;
        }

        @NotNull
        @Override
        public Iterator<PathPoint> iterator() {
            return this;
        }

    }

    // -----------------------------------------------------------------------------------------------------------------

    /**
     * A follower that will generate points along the path from start to finish as described by the
     * parametric position on the path
     */
    public class PathFollower extends PathGenerator {
        private PathFollower() {
            resetSegment();
        }

        public PathPoint getPointAt(double time) {
            // get the next point on the curve
            if (m_thisSegmentStart == null) {
                return null;
            }
            while (time > m_thisSegmentEnd.m_time) {
                m_thisSegmentStart = m_thisSegmentEnd;
                m_thisSegmentEnd = m_thisSegmentStart.m_next;
                if (m_thisSegmentEnd == null) {
                    return null;
                }
                resetSegment();
            }

            // create and return the path point
            return getPointOnSegment(time - m_thisSegmentStart.m_time);
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // This is the actual implementation of the spline object - which is really just a manager of the control
    // points of the spline and a factor for the PathIterator and PathFollower.

    /**
     *
     */
    public KochanekBartelsSpline() {
    }

    public ControlPoint addControlPoint(Point2D pt) {
        return addControlPoint(pt.getX(), pt.getY(), 0.0);
    }

    public ControlPoint addControlPoint(double fieldX, double fieldY) {
        return addControlPoint(fieldX, fieldY, 0.0);
    }

    public ControlPoint addControlPoint(double fieldX, double fieldY, double fieldHeading) {
        ControlPoint newControlPoint = new ControlPoint((null == m_last) ? 0.0 : m_last.m_time + 1.0);
        if (null == m_first) {
            m_first = newControlPoint;
        } else {
            m_last.m_next = newControlPoint;
            newControlPoint.m_last = m_last;
        }
        m_last = newControlPoint;
        newControlPoint.setFieldLocation(fieldX, fieldY);
        newControlPoint.updateHeadingDerivative();
        return newControlPoint;
    }

    public Iterable<ControlPoint> getControlPoints() {
        return new ControlPointIterator();
    }

    public Iterable<PathPoint> getCurveSegments() {
        return new PathIterator(0.05);
    }

    public PathFollower getPathFollower() {
        return new PathFollower();
    }

}
