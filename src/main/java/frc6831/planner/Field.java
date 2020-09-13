package frc6831.planner;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;

/**
 * This class maintains and draws the field we will be plotting the path on. Note that the default with no loaded
 * field data is to draw the axes and a dotted outline of the field.
 */
public class Field {

    private final double X_AXIS_MIN = -4.5;
    private final double X_AXIS_MAX = 4.5;
    private final double Y_AXIS_MIN = -8.5;
    private final double Y_AXIS_MAX = 8.5;
    private final double X_FIELD_MIN = -4.105;
    private final double X_FIELD_MAX = 4.105;
    private final double Y_FIELD_MIN = -7.99;
    private final double Y_FIELD_MAX = 7.99;

    private final Point2D.Double X_AXIS_START = new Point2D.Double(0.0, Y_AXIS_MIN);
    private final Point2D.Double X_AXIS_END = new Point2D.Double(0.0, Y_AXIS_MAX);
    private final Point2D.Double Y_AXIS_START = new Point2D.Double(X_AXIS_MIN, 0.0);
    private final Point2D.Double Y_AXIS_END = new Point2D.Double(X_AXIS_MAX, 0.0);

    private final Point2D.Double[] FIELD_OUTLINE = {
            new Point2D.Double(X_FIELD_MIN, Y_FIELD_MIN),
            new Point2D.Double(X_FIELD_MIN, Y_FIELD_MAX),
            new Point2D.Double(X_FIELD_MAX, Y_FIELD_MAX),
            new Point2D.Double(X_FIELD_MAX, Y_FIELD_MIN)
    };
    private final Point2D.Double[] m_xfmField = {
            new Point2D.Double(X_FIELD_MIN, Y_FIELD_MIN),
            new Point2D.Double(X_FIELD_MIN, Y_FIELD_MAX),
            new Point2D.Double(X_FIELD_MAX, Y_FIELD_MAX),
            new Point2D.Double(X_FIELD_MAX, Y_FIELD_MIN)
    };

    public class MinMax {
        protected float m_minX;
        protected float m_minY;
        protected float m_maxX;
        protected float m_maxY;

        public MinMax(float minX, float minY, float maxX, float maxY) {
            m_minX = minX;
            m_minY = minY;
            m_maxX = maxX;
            m_maxY = maxY;
        }

        public float getMinX() {
            return m_minX;
        }

        public float getMinY() {
            return m_minY;
        }

        public float getMaxX() {
            return m_maxX;
        }

        public float getMaxY() {
            return m_maxY;
        }
    }

    private MinMax m_minMax = new MinMax(-4.5f, -8.5f, 4.5f, 8.5f);

    public MinMax getMinMax() {
        return m_minMax;
    }

    public void draw(Graphics2D g2d, AffineTransform drawXfm) {
        Stroke oldStroke = g2d.getStroke();

        // draw the axis
        drawLine(g2d, drawXfm, X_AXIS_START, X_AXIS_END);
        drawLine(g2d, drawXfm, Y_AXIS_START, Y_AXIS_END);

        // draw the field outline as a dotted line
        drawPolyLine(g2d, drawXfm, FIELD_OUTLINE, m_xfmField, true);

        // now draw the field that was read in from the field data file.

        g2d.setStroke(oldStroke);
    }

    private void drawLine(Graphics2D g2d, AffineTransform drawXfm, Point2D.Double start, Point2D.Double end) {
        Point2D.Double ptStart = (Point2D.Double) drawXfm.transform(start, null);
        Point2D.Double ptEnd = (Point2D.Double) drawXfm.transform(end, null);
        g2d.drawLine((int) ptStart.getX(), (int) ptStart.getY(), (int) ptEnd.getX(), (int) ptEnd.getY());
    }
    private void drawPolyLine(Graphics2D g2d, AffineTransform drawXfm,
                              Point2D.Double[] fieldPts, Point2D.Double[] xfmPts, boolean closed) {
        drawXfm.transform(fieldPts, 0, xfmPts, 0, fieldPts.length);
        Point2D.Double lastPt = closed ? xfmPts[fieldPts.length-1] : null;
        for (int n = 0; n < fieldPts.length; n++) {
            Point2D.Double thisPt = xfmPts[n];
            if (null != lastPt) {
                g2d.drawLine((int) thisPt.getX(), (int) thisPt.getY(), (int) lastPt.getX(), (int) lastPt.getY());
            }
            lastPt = thisPt;
        }
    }
}
