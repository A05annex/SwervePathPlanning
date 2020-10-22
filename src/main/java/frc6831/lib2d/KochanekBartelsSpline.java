package frc6831.lib2d;

import org.jetbrains.annotations.NotNull;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.ParseException;

import java.awt.geom.Point2D;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.Iterable;
import java.util.Iterator;

import static frc6831.lib2d.JsonSupport.*;

/**
 * This is an implementation of the
 * <a href="https://en.wikipedia.org/wiki/Kochanek%E2%80%93Bartels_spline">Kochanek-Bartels Spline</a> designed
 * for interactive editing of the tangent vector to implicitly control bias and tension. There is no continuity
 * control as we expect robot movement to be continuous unless we end this spline and the robot stops to do
 * something.
 * <p>
 * When control points are created the tangent (derivatives) at that control point and surrounding control points are
 * computed using the <a href="https://en.wikipedia.org/wiki/Cubic_Hermite_spline">Cardinal-Spline</a> formulation
 * with the default tension specified by {@link #DEFAULT_TENSION}. The tangent is adjusted using a control handle
 * which intuitively manipulates the shape of the spline at the control point to implicitly edit tension and bias.
 * <p>
 * This class is primarily a container-editor for a doubly-linked list of control points, and a factory for
 * path point iterators (iterating through the path points), or path point followers (generating a series of
 * path points at specified times along the path).
 */
public class KochanekBartelsSpline {

    // -----------------------------------------------------------------------------------------------------------------
    // these are the keys for the JSON representation of the spline.
    static final String TITLE = "title";
    static final String DESCRIPTION = "description";
    static final String CONTROL_POINTS = "controlPoints";
    static final String FIELD_X = "fieldX";
    static final String FIELD_Y = "fieldY";
    static final String FIELD_HEADING = "fieldHeading";
    static final String TIME = "time";
    static final String LOCATION_DERIVATIVES_EDITED = "derivativesEdited";
    static final String FIELD_dX = "field_dX";
    static final String FIELD_dY = "field_dY";
    static final String FIELD_dHEADING = "field_dHeading";

    // -----------------------------------------------------------------------------------------------------------------
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

    private static final double DEFAULT_PATH_DELTA = 0.05;

    private static final String DEFAULT_TITLE = "untitled";

    private static final String DEFAULT_DESCRIPTION = "No description provided.";

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

    private String m_title = DEFAULT_TITLE;
    private String m_description = DEFAULT_DESCRIPTION;
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
    // forward speed, strafe speed, and rotation speed for the robot.
    // -----------------------------------------------------------------------------------------------------------------

    /**
     * The representation of a generated point along the path. The path points is expressed as both the field
     * position and heading, and the derivative (velocities) of the field position and heading. The derivatives
     * are transformed to be robot-relative so they can be used to set the robot speeds for following this path.
     */
    public static class PathPoint {
        public final ControlPoint previousControlPoint;
        public final ControlPoint nextControlPoint;
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
         *
         * @param fieldX               The expected field X position of the robot in meters.
         * @param fieldY               The expected field Y position of the robot in meters.
         * @param fieldHeading         The expected heading of the robot in radians.
         * @param speedForward         The forward chassis speed of the robot in meters/sec.
         * @param speedStrafe          The strafe chassis velocity of the robot in meters/sec.
         * @param speedRotation        The rotation speed of the robot in radians/sec.
         * @param previousControlPoint (not null) The control point at the start of the curve segment
         *                             containing this path point.
         * @param nextControlPoint     (not null) The control point at the end of the curve segment
         *                             containing this path point.
         */
        public PathPoint(double fieldX, double fieldY, double fieldHeading,
                         double speedForward, double speedStrafe, double speedRotation,
                         @NotNull ControlPoint previousControlPoint, @NotNull ControlPoint nextControlPoint) {
            this.previousControlPoint = previousControlPoint;
            this.nextControlPoint = nextControlPoint;
            this.fieldPt = new Point2D.Double(fieldX, fieldY);
            this.fieldHeading = fieldHeading;
            this.speedForward = speedForward;
            this.speedStrafe = speedStrafe;
            this.speedRotation = speedRotation;
        }

        /**
         * Test whether a field position (probably a mouse position during path editing) is over this path
         * point.
         *
         * @param fieldX    (double) The X coordinate of the test point.
         * @param fieldY    (double) The Y coordinate of the test point.
         * @param tolerance (double) The test tolerance - specifically, the distance from the actual field
         *                  position that the test point must be within to be considered a hit on the
         *                  path point.
         * @return Returns {@code true} if the test point is over the path point, and {@code false} otherwise.
         */
        public boolean testOverPathPoint(double fieldX, double fieldY, double tolerance) {
            double dx = fieldPt.getX() - fieldX;
            double dy = fieldPt.getY() - fieldY;
            return Math.sqrt((dx * dx) + (dy * dy)) < tolerance;
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
        ControlPoint m_next = null;
        ControlPoint m_last = null;
        double m_fieldX = 0.0;
        double m_fieldY = 0.0;
        double m_fieldHeading = 0.0;
        double m_time;
        boolean m_locationDerivativesEdited = false;
        double m_dX = 0.0;
        double m_dY = 0.0;
        double m_dHeading = 0.0;

        /**
         * Instantiate a control point and set the time this control point should be reached when the path
         * is traversed.
         *
         * @param timeInSec (double) The time this control point should be reached (in seconds).
         */
        public ControlPoint(double timeInSec) {
            m_time = timeInSec;
        }

        public ControlPoint(JSONObject json) {
            m_fieldX = parseDouble(json, FIELD_X, 0.0);
            m_fieldY = parseDouble(json, FIELD_Y, 0.0);
            m_fieldHeading = parseDouble(json, FIELD_HEADING, 0.0);
            m_time = parseDouble(json, TIME, 0.0);
            m_locationDerivativesEdited = parseBoolean(json, LOCATION_DERIVATIVES_EDITED, false);
            m_dX = parseDouble(json, FIELD_dX, 0.0);
            m_dY = parseDouble(json, FIELD_dY, 0.0);
            m_dHeading = parseDouble(json, FIELD_dHEADING, 0.0);
        }

        @SuppressWarnings("unchecked")
        public @NotNull JSONObject toJSON() {
            JSONObject controlPoint = new JSONObject();
            controlPoint.put(FIELD_X, m_fieldX);
            controlPoint.put(FIELD_Y, m_fieldY);
            controlPoint.put(FIELD_HEADING, m_fieldHeading);
            controlPoint.put(TIME, m_time);
            controlPoint.put(LOCATION_DERIVATIVES_EDITED, m_locationDerivativesEdited);
            controlPoint.put(FIELD_dX, m_dX);
            controlPoint.put(FIELD_dY, m_dY);
            controlPoint.put(FIELD_dHEADING, m_dHeading);
            return controlPoint;
        }

        /**
         * Restore the control point to automated derivative recalculation when it or surrounding
         * control points are moved. This only effects points whosee derivatives have been manually edited.
         */
        public void resetDerivative() {
            if (m_locationDerivativesEdited) {
                m_locationDerivativesEdited = false;
                updateLocationDerivatives();
                updateHeadingDerivative();
            }
        }

        public boolean getDerivativesManuallyEdited() {
            return m_locationDerivativesEdited;
        }

        public double getFieldX() {
            return m_fieldX;
        }

        public double getFieldY() {
            return m_fieldY;
        }

        /**
         * Set the field location of a control point to the specified point. This method forwards the request
         * to {@link #setFieldLocation(double, double)} which documents the actual behaviour.
         *
         * @param pt (not null, Point2D) The new field location for the point.
         */
        public void setFieldLocation(Point2D pt) {
            setFieldLocation(pt.getX(), pt.getY());
        }

        /**
         * Set the field location of a control point to the specified point {@code fieldX, fieldY}
         * position. This method is normally called during an interactive manipulation of the location
         * handle. When the position of a control point is updated the derivatives of that point and the
         * control points before and after that point are recomputed. If the derivatives for any of those
         * points has been previously manually edited, the manually edited derivatives are preserved.
         *
         * @param fieldX (double) The new field X position.
         * @param fieldY (double) The new field Y position.
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
         * Recompute the derivatives for this control point. If the derivatives have been previously manually
         * edited this request is ignored and the manually edited derivatives are preserved.
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
         * Get the field X location for the display of af a tangent editing handle for this control point.
         *
         * @return (double) The field X location of the handle for editing the tangent of the control point.
         */
        public double getTangentX() {
            return m_fieldX + (DERIVATIVE_UI_SCALE * m_dX);
        }

        /**
         * Get the field X location for the display of af a tangent editing handle for this control point.
         *
         * @return (double) The field X location of the handle for editing the tangent of the control point.
         */
        public double getTangentY() {
            return m_fieldY + (DERIVATIVE_UI_SCALE * m_dY);
        }

        /**
         * Set the tangent location, used during interactive curve editing. This method forwards the request
         * to {@link #setTangentLocation(double, double)} which documents the actual behaviour.
         *
         * @param pt (not null, Point2D) The new field location for the tangent editing handle.
         */
        public void setTangentLocation(Point2D pt) {
            setTangentLocation(pt.getX(), pt.getY());
        }

        /**
         * Set the tangent location, used during interactive curve editing. This method is normally called during
         * an interactive manipulation of the tangent handle. When the tangent is manipulated through tis method
         * it is marked as manually manipulated, and it will not be automatically updated as this or adjacent
         * control points are moved.
         *
         * @param fieldX (double) The new field X position of the tangent handle.
         * @param fieldY (double) The new field Y position of the tangent handle.
         */
        public void setTangentLocation(double fieldX, double fieldY) {
            m_dX = (fieldX - m_fieldX) / DERIVATIVE_UI_SCALE;
            m_dY = (fieldY - m_fieldY) / DERIVATIVE_UI_SCALE;
            m_locationDerivativesEdited = true;
        }

        public double getFieldHeading() {
            return m_fieldHeading;
        }

        /**
         * Get the field X location for the display of af a heading editing handle for this control point.
         *
         * @return (not null, double) The field X location for the heading editing handle.
         */
        public double getHeadingX() {
            return m_fieldX + (ROBOT_HEADING_HANDLE * Math.sin(m_fieldHeading));
        }

        /**
         * Get the field Y location for the display of af a heading editing handle for this control point.
         *
         * @return (not null, double) The field Y location for the heading editing handle.
         */
        public double getHeadingY() {
            return m_fieldY + (ROBOT_HEADING_HANDLE * Math.cos(m_fieldHeading));
        }

        /**
         * Set the robot heading vector to go through the specified point at this control point. NOTE: the
         * heading handle/display is a fixed distance from the centroid of the robot. The specified {@code pt}
         * is taken to be a point somewhere along the heading vector, and the heading handle will be repositioned
         * to be on that heading vector.
         *
         * @param pt (not null, Point2D) A point through which the heading vector from the centroid of the
         *           robot should pass.
         */
        public void setHeadingLocation(Point2D pt) {
            // OK, the simple action here is to look at the current mouse position relative to the control
            // point position, use the atan2, and get a heading. However, tis does not handle the -180/180 degree
            // transition, so we need some logic like the NavX logic. for passing over the boundary
            setFieldHeading(Math.atan2(pt.getX() - m_fieldX, pt.getY() - m_fieldY));
        }

        /**
         * Set the heading direction for this control point.
         *
         * @param heading (double) The heading direction, in radians, for this control point.
         */
        public void setFieldHeading(double heading) {
            while ((heading - m_fieldHeading) > Math.PI) {
                heading -= 2.0 * Math.PI;
            }
            while ((heading - m_fieldHeading) < -Math.PI) {
                heading += 2.0 * Math.PI;
            }
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
         * Test whether a field position (probably a mouse position during path editing) is over this control
         * point.
         *
         * @param fieldX    (double) The X coordinate of the test point.
         * @param fieldY    (double) The Y coordinate of the test point.
         * @param tolerance (double) The test tolerance - specifically, the distance from the actual field
         *                  position that the test point must be within to be considered a hit on the
         *                  control point position.
         * @return Returns {@code true} if the test X,Y is over the control point, and {@code false} otherwise.
         */
        public boolean testOverControlPoint(double fieldX, double fieldY, double tolerance) {
            double dx = m_fieldX - fieldX;
            double dy = m_fieldY - fieldY;
            return Math.sqrt((dx * dx) + (dy * dy)) < tolerance;
        }

        /**
         * Test whether a field position (probably a mouse position during path editing) is over this control
         * point tangent editing handle.
         *
         * @param fieldX    (double) The X coordinate of the test point.
         * @param fieldY    (double) The Y coordinate of the test point.
         * @param tolerance (double) The test tolerance - specifically, the distance from the actual field
         *                  position that the test point must be within to be considered a hit on the
         *                  control point tangent handle.
         * @return Returns {@code true} if the test point is over the tangent point, and {@code false} otherwise.
         */
        public boolean testOveTangentPoint(double fieldX, double fieldY, double tolerance) {
            double dx = getTangentX() - fieldX;
            double dy = getTangentY() - fieldY;
            return Math.sqrt((dx * dx) + (dy * dy)) < tolerance;
        }

        /**
         * Test whether a field position (probably a mouse position during path editing) is over this control
         * point robot heading control handle.
         *
         * @param fieldX    (double) The X coordinate of the test point.
         * @param fieldY    (double) The Y coordinate of the test point.
         * @param tolerance (double) The test tolerance - specifically, the distance from the actual field
         *                  position that the test point must be within to be considered a hit on the
         *                  control point heading handle.
         * @return Returns {@code true} if the test point is over the heading point, and {@code false} otherwise.
         */
        public boolean testOverHeadingPoint(double fieldX, double fieldY, double tolerance) {
            double dx = getHeadingX() - fieldX;
            double dy = getHeadingY() - fieldY;
            return Math.sqrt((dx * dx) + (dy * dy)) < tolerance;
        }
    }

    // -----------------------------------------------------------------------------------------------------------------

    /**
     * This class is an iterator over the control points of the path. It is normally used in path editing
     * programs that needs to display some representation of the control points.
     */
    public class ControlPointIterator implements Iterator<ControlPoint>, Iterable<ControlPoint> {

        private ControlPoint m_current = m_first;

        /**
         * Instantiate the control point iterator.
         */
        private ControlPointIterator() {
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

    /**
     * This is an abstract class that handles the mechanics of generating points along a path described by this
     * spline. This base class assumes that points along the path will be evaluated with an ever increasing
     * time and support a {@link PathIterator} that generates points at a defined time interval (useful in path
     * display during planning), and {@link PathFollower} that generates points at the requested time (useful
     * in animation or when the robot is in a command loop to generate the points at whatever time they are
     * requested).
     * <p>
     * The path is divided into segments, each having a start and end control point. As points are generated
     * this class has a method to update the segment start-end parameters whenever a control point boundary
     * is passed. The implementing class is responsible for calling this method at the appropriate time.
     */
    private abstract class PathGenerator {
        /**
         * The start of the segment being generated, will be {@code null} if no control points have been defined.
         */
        ControlPoint m_thisSegmentStart = m_first;
        /**
         * The end of the segment being generated, will be{@code null} if less than 2 control points have been
         * defined.
         */
        ControlPoint m_thisSegmentEnd = m_first == null ? null : m_first.m_next;
        /**
         * The control point location, heading, derivatives matrix for this segment of
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

        /**
         * Generate a {@link PathPoint} in the context of a path iterator or path follower for this spline
         * for the specified time along the path. NOTE: The time must be monotonically increasing
         *
         * @param time The time, in seconds, along the path for which the {@link PathPoint} will be generated.
         * @return Returns the {@link PathPoint} for the specified time, or {@link null} if the time is beyond
         * the last control point in the path.
         */
        public PathPoint getPointOnSegment(double time) {

            while (time > m_thisSegmentEnd.m_time) {
                // past the end of this segment, move on to the next.
                m_thisSegmentStart = m_thisSegmentEnd;
                m_thisSegmentEnd = m_thisSegmentStart.m_next;
                if (null == m_thisSegmentEnd) {
                    // No more segments, we are done.
                    return null;
                }
                resetSegment();
            }
            // create and return the path point
            double sValue = time - m_thisSegmentStart.m_time;
            // get the next point on the curve
            // The s[] vector is s to the third, second, first, and 0th power
            double[] s = {sValue * sValue * sValue, sValue * sValue, sValue, 1.0};
            // the ds[] vector is the derivative of the s[] vector
            double[] ds = {3.0 * sValue * sValue, 2.0 * sValue, 1.0, 0.0};
            // This first transformation multiples the s and derivative s matrices by the
            // the basis functions to get the weights that are applied to the segment start and
            // end point
            double[] weights = {0.0, 0.0, 0.0, 0.0};
            double[] dWeights = {0.0, 0.0, 0.0, 0.0};
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    weights[i] += s[j] * m_basis[j][i];
                    dWeights[i] += ds[j] * m_basis[j][i];
                }
            }
            // Now that we have the weights, we multiply the field positions and derivatives of the
            // start and end control points by the weights to get the expected field position-heading and
            // the derivatives - the calculated robot forward, strafe, and rotation speeds necessary to
            // that should be happening at this point.
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
            return new PathPoint(field[0], field[1], field[2], forward, strafe, dField[2],
                    m_thisSegmentStart, m_thisSegmentEnd);
        }
    }

    /**
     * An iterator for points along a path at a specified time interval.
     */
    public class PathIterator extends PathGenerator implements Iterator<PathPoint>, Iterable<PathPoint> {
        /**
         * The current position on the segment being generated, from 0.0 being on {@link #m_thisSegmentStart}
         * to 1.0 being on {@link #m_thisSegmentEnd}.
         */
        double m_time;
        /**
         * The point spacing increment on the curve.
         */
        final double m_deltaTime;

        /**
         * Instantiate a path iterator that will generate points on the path at the specified time interval.
         *
         * @param deltaTime The ime interval, in seconds, at which the iterator will return {@link PathPoint}s.
         */
        private PathIterator(double deltaTime) {
            resetSegment();
            m_time = 0.0;
            m_deltaTime = deltaTime;
        }


        @Override
        public boolean hasNext() {
            return (null != m_thisSegmentEnd) && (m_time <= m_last.m_time);
        }

        @Override
        public PathPoint next() {
            // get the next point on the curve
            PathPoint pathPoint = getPointOnSegment(m_time);
            // get ready for the next point
            m_time += m_deltaTime;
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
     * time for the point on the path. The time must be monotonically increasing. The follower is designed
     * for use in the robot control loop where the interval between calls may vary.
     */
    public class PathFollower extends PathGenerator {
        /**
         * Instantiate a path follower.
         */
        private PathFollower() {
            resetSegment();
        }

        /**
         * Get the path point at the specified time.
         *
         * @param time The time, in seconds, along the path for which the {@link PathPoint} will be generated.
         * @return Returns the {@link PathPoint} for the specified time. Returns {@code null} if the time is
         * past the end of the end of the path (the time of the last control point on the path)..
         */
        public PathPoint getPointAt(double time) {
            // get the next point on the curve
            if (m_thisSegmentEnd == null) {
                return null;
            }
            // create and return the path point
            return getPointOnSegment(time);
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // This is the actual implementation of the spline object - which is really just a manager of the control
    // points of the spline and a factory for the PathIterator and PathFollower.
    // -----------------------------------------------------------------------------------------------------------------

    /**
     * The implementation of the spline object. The spline object that manages the set of control points and
     * is a factory for path iterators and path followers.
     */
    public KochanekBartelsSpline() {
    }

    public void setTitle(@NotNull String title) {
        m_title = title;
    }

    public String getTitle() {
        return m_title;
    }

    public void setDescription(@NotNull String description) {
        m_description = description;
    }

    public String getDescription() {
        return m_description;
    }

    /**
     * Add a control point to the end of the path, which will extend that path to that new control point. This
     * method defers to {@link #addControlPoint(double, double, double)} with a heading of 0.0.
     *
     * @param pt The field location of the new control point.
     * @return (not null) Returns the added control point.
     */
    @NotNull
    public ControlPoint addControlPoint(@NotNull Point2D pt) {
        return addControlPoint(pt.getX(), pt.getY(), 0.0);
    }

    /**
     * Add a control point to the end of the path, which will extend that path to that new control point. This
     * method defers to {@link #addControlPoint(double, double, double)} with a heading of 0.0.
     *
     * @param fieldX The X field position for the added control point.
     * @param fieldY The field Y position for the added control point.
     * @return (not null) Returns the added control point.
     */
    @NotNull
    public ControlPoint addControlPoint(double fieldX, double fieldY) {
        return addControlPoint(fieldX, fieldY, 0.0);
    }

    /**
     * Add a control point to the end of the path, which will extend that path to that new control point.
     *
     * @param fieldX       The X field position for the added control point.
     * @param fieldY       The field Y position for the added control point.
     * @param fieldHeading The heading for the added control point.
     * @return (not null) Returns the added control point.
     */
    @NotNull
    public ControlPoint addControlPoint(double fieldX, double fieldY, double fieldHeading) {
        ControlPoint newControlPoint = new ControlPoint((null == m_last) ? 0.0 : m_last.m_time + 1.0);
        if (null == m_first) {
            m_first = newControlPoint;
        }
        if (null != m_last) {
            m_last.m_next = newControlPoint;
            newControlPoint.m_last = m_last;
        }
        m_last = newControlPoint;
        newControlPoint.setFieldLocation(fieldX, fieldY);
        newControlPoint.setFieldHeading(fieldHeading);
        return newControlPoint;
    }

    /**
     * Clear the path to an empty path with no control points.
     */
    public void clearPath() {
        m_title = DEFAULT_TITLE;
        m_description = DEFAULT_DESCRIPTION;
        m_first = null;
        m_last = null;
    }

    /**
     * Insert a control point at a specific field position and heading before an existing control point. When
     * a control point is inserted the time for the following points is shifted to compensate for the insertion.
     *
     * @param controlPoint (not null) The existing control point the new control point will be inserted before.
     * @param fieldX       The X field position for the added control point.
     * @param fieldY       The field Y position for the added control point.
     * @param fieldHeading The heading for the added control point.
     * @return (not null) Returns the added control point.
     */
    @NotNull
    public ControlPoint insertControlPointBefore(@NotNull ControlPoint controlPoint,
                                                 double fieldX, double fieldY, double fieldHeading) {
        ControlPoint newControlPoint = new ControlPoint(controlPoint.m_time);
        newControlPoint.m_last = controlPoint.m_last;
        if (null == newControlPoint.m_last) {
            // there is no last control point because this is being inserted in front of the first point
            m_first = newControlPoint;
        } else {
            newControlPoint.m_last.m_next = newControlPoint;
        }
        newControlPoint.m_next = controlPoint;
        controlPoint.m_last = newControlPoint;
        // reset timing on all control points after this control point
        ControlPoint tmpControlPoint = newControlPoint;
        while (null != tmpControlPoint) {
            if (null != tmpControlPoint.m_next) {
                tmpControlPoint.m_time = tmpControlPoint.m_next.m_time;
            } else {
                tmpControlPoint.m_time += 1.0;
            }
            tmpControlPoint = tmpControlPoint.m_next;
        }
        // set the location and heading for this control point
        newControlPoint.setFieldLocation(fieldX, fieldY);
        newControlPoint.setFieldHeading(fieldHeading);
        return newControlPoint;
    }

    /**
     * Delete the specified control point. When a control point is deleted the time for the following points
     * is shifted to compensate for the deletion.
     *
     * @param controlPoint (not null) The control point to be deleted.
     */
    public void deleteControlPoint(@NotNull ControlPoint controlPoint) {
        // shift the time of any point past the one to be deleted.
        ControlPoint tmpControlPoint = m_last;
        while (controlPoint != tmpControlPoint) {
            tmpControlPoint.m_time = tmpControlPoint.m_last.m_time;
            tmpControlPoint = tmpControlPoint.m_last;
        }

        // now delete the point (remove it from the list)
        if (null != controlPoint.m_last) {
            controlPoint.m_last.m_next = controlPoint.m_next;
        } else {
            // this s the first point being deleted
            m_first = controlPoint.m_next;
        }
        if (null != controlPoint.m_next) {
            controlPoint.m_next.m_last = controlPoint.m_last;
        } else {
            // This is the last point being deleted
            m_last = controlPoint.m_last;
        }

        // and reset the derivatives for the surrounding points.
        if (controlPoint.m_last != null) {
            controlPoint.m_last.updateLocationDerivatives();
            controlPoint.m_last.updateHeadingDerivative();
        }
        if (controlPoint.m_next != null) {
            controlPoint.m_next.updateLocationDerivatives();
            controlPoint.m_next.updateHeadingDerivative();
        }
    }

    /**
     * This factory method instantiates a control point iterator.
     *
     * @return (not null) Returns an iterator for the control points of this spline.
     */
    @NotNull
    public Iterable<ControlPoint> getControlPoints() {
        return new ControlPointIterator();
    }


    /**
     * @return
     */
    @NotNull
    public Iterable<PathPoint> getCurveSegments() {
        return new PathIterator(DEFAULT_PATH_DELTA);
    }

    /**
     * @param timeInterval
     * @return
     */
    @NotNull
    public Iterable<PathPoint> getCurveSegments(double timeInterval) {
        return new PathIterator(timeInterval);
    }

    /**
     * @return
     */
    @NotNull
    public PathFollower getPathFollower() {
        return new PathFollower();
    }

    /**
     * Load a path from a {@code .json} path file.
     *
     * @param filename (String, not null) The name of the file the path will be read from.
     */
    public void loadPath(String filename) {
        // The deal here is that we want to make sure we can read this file as a path before we
        // the existing path description.
        clearPath();
        try {
            JSONObject path = readJsonFile(filename);
            m_title = parseString(path, TITLE, DEFAULT_TITLE);
            m_description = parseString(path, DESCRIPTION, DEFAULT_DESCRIPTION);
            JSONArray controlPoints = getJSONArray(path, CONTROL_POINTS);
            for (Object cpObj : controlPoints) {
                JSONObject cpJson = (JSONObject) cpObj;
                ControlPoint newControlPoint = new ControlPoint(cpJson);
                if (null == m_first) {
                    m_first = newControlPoint;
                }
                if (null != m_last) {
                    m_last.m_next = newControlPoint;
                    newControlPoint.m_last = m_last;
                }
                m_last = newControlPoint;
            }


        } catch (IOException | ParseException | ClassCastException | NullPointerException e) {
            e.printStackTrace();
        }

    }

    /**
     * Save the path to a {@code .json} path file.
     *
     * @param filename (String, not null) The filename the path will be written to.
     */
    @SuppressWarnings("unchecked")
    public void savePath(@NotNull String filename) {
        JSONObject path = new JSONObject();
        path.put(TITLE, m_title);
        path.put(DESCRIPTION, m_description);
        JSONArray controlPoints = new JSONArray();
        path.put(CONTROL_POINTS, controlPoints);
        for (ControlPoint pt : getControlPoints()) {
            controlPoints.add(pt.toJSON());
        }
        //Write JSON file
        try (FileWriter file = new FileWriter(filename)) {
            file.write(path.toJSONString());
            file.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
