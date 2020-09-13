package frc6831.planner;

import frc6831.lib2d.KochanekBartelsSpline;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.GeneralPath;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.geom.Point2D;
import java.awt.image.ColorModel;

/**
 * This is the canvas we draw the field and path to. It is derived from a
 * {@link Canvas} so it has all the mechanics to support resizing, mouse interaction,
 * and 2D drawing.
 */
public class PathCanvas extends Canvas {

    // constants to manage the interaction
    private static final double OVER_TOL = 5.0;
    private static final String MODE_ADD = "add";
    private static final String MODE_EDIT = "edit";
    private static final String OVER_CONTROL_POINT = "controlPoint";
    private static final String OVER_TANGENT_POINT = "tangentPoint";
    private static final String OVER_HEADING_POINT = "headingPoint";

    private static final double ROBOT_WIDTH = 0.762;
    private static final double ROBOT_WIDTH_BUMPER = 0.9144;
    private static final double ROBOT_LENGTH = 0.762;
    private static final double ROBOT_LENGTH_BUMPER = 0.9144;

    // the actual data for the field and path
    final private Field m_field = new Field();
    private KochanekBartelsSpline path = new KochanekBartelsSpline();
    private AffineTransform m_drawXfm = null;
    private AffineTransform m_mouseXfm = null;
    private double m_scale;

    final private GeneralPath robot;
    final private GeneralPath robotBumpers;

    // controlling the user interaction with the
    private KochanekBartelsSpline.ControlPoint newControlPoint = null;
    private KochanekBartelsSpline.ControlPoint overControlPoint = null;
    private String m_mode = MODE_ADD;
    private String m_overWhat = null;
    private Stroke m_highlightStroke = new BasicStroke(2.0f);
    private Point2D.Double m_mouse = null;

    static private class ScalingGraphicsConfig extends GraphicsConfiguration {

        private final GraphicsConfiguration m_gc;

        ScalingGraphicsConfig(GraphicsConfiguration gc) {
            m_gc = gc;
        }

        @Override
        public GraphicsDevice getDevice() {
            return m_gc.getDevice();
        }

        @Override
        public ColorModel getColorModel() {
            return m_gc.getColorModel();
        }

        @Override
        public ColorModel getColorModel(int transparency) {
            return m_gc.getColorModel();
        }

        @Override
        public AffineTransform getDefaultTransform() {
            return AffineTransform.getScaleInstance(4.0,-4.0);
//            return m_gc.getDefaultTransform();
        }

        @Override
        public AffineTransform getNormalizingTransform() {
            return m_gc.getNormalizingTransform();
        }

        @Override
        public Rectangle getBounds() {
            return m_gc.getBounds();
        }
    }

    /**
     * This is the handler for resizing. The main thing in resizing is that we scale the
     * drawing so the field fills the window but maintains the correct field aspect ratio.
     */
    private class ComponentHandler extends ComponentAdapter {
        public void componentResized(ComponentEvent e) {
            Component comp = e.getComponent();
            float width = comp.getWidth();
            float height = comp.getHeight();
            System.out.println(String.format("Size Changed %d,%d", (int)width,(int)height));
            // OK, so here we pick whether we scale X or Y to fill the window, reverse Y,
            // and, translate 0,0 to center screen.
            Field.MinMax fieldMinMax = m_field.getMinMax();
            float scaleX = width / (fieldMinMax.m_maxX - fieldMinMax.m_minX);
            float scaleY = height / (fieldMinMax.m_maxY - fieldMinMax.m_minY);
            m_scale = (scaleX < scaleY) ? scaleX : scaleY;
            m_drawXfm = new AffineTransform(m_scale, 0.0f, 0.0f, -m_scale, width/2.0f,height/2.0f);
            m_mouseXfm = new AffineTransform(m_drawXfm);
            try {
                m_mouseXfm.invert();
            } catch (NoninvertibleTransformException ex) {
                System.out.println("  -- can't invert draw transform");
            }
        }

    }

    private class MouseHandler extends MouseAdapter {
        @Override
        public void mousePressed(MouseEvent e) {
            Point2D pt = m_mouse = (Point2D.Double) m_mouseXfm.transform(
                    new Point2D.Double(e.getPoint().getX(),e.getPoint().getY()), null);
            if ((m_mode == MODE_ADD) && (e.getClickCount() == 1)) {
                newControlPoint = path.addControlPoint(pt);
                repaint();
            } else if (m_mode == MODE_EDIT) {
                overControlPoint = null;
                m_overWhat = null;
                for (KochanekBartelsSpline.ControlPoint point : path.getControlPoints()) {
                    if (point.testOverControlPoint(pt.getX(), pt.getY(), OVER_TOL/m_scale)) {
                        overControlPoint = point;
                        m_overWhat = OVER_CONTROL_POINT;
                    } else if (point.testOveTangentPoint(pt.getX(), pt.getY(), OVER_TOL/m_scale)) {
                        overControlPoint = point;
                        m_overWhat = OVER_TANGENT_POINT;
                    } else if (point.testOveHeadingPoint(pt.getX(), pt.getY(), OVER_TOL/m_scale)) {
                        overControlPoint = point;
                        m_overWhat = OVER_HEADING_POINT;
                    }
                }
                repaint();
            }
        }

        @Override
        public void mouseReleased(MouseEvent e) {
            Point2D pt = m_mouse = (Point2D.Double) m_mouseXfm.transform(
                    new Point2D.Double(e.getPoint().getX(),e.getPoint().getY()), null);
            if (m_mode == MODE_ADD)  {
                if (e.getClickCount() == 1) {
                    newControlPoint = null;
                    repaint();
                } else if (e.getClickCount() == 2) {
                    m_mode = MODE_EDIT;
                }
            }
        }

        @Override
        public void mouseEntered(MouseEvent e) {
        }

        @Override
        public void mouseExited(MouseEvent e) {
        }

        @Override
        public void mouseWheelMoved(MouseWheelEvent e) {
        }

        @Override
        public void mouseDragged(MouseEvent e) {
            Point2D pt = m_mouse = (Point2D.Double) m_mouseXfm.transform(
                    new Point2D.Double(e.getPoint().getX(),e.getPoint().getY()), null);
            if (m_mode == MODE_ADD) {
                newControlPoint.setFieldLocation(pt);
            } else if ((m_mode == MODE_EDIT) && (null != overControlPoint)) {
                if (OVER_CONTROL_POINT == m_overWhat) {
                    overControlPoint.setFieldLocation(pt);
                } else if (OVER_TANGENT_POINT == m_overWhat) {
                    overControlPoint.setTangentLocation(pt);
                } else if (OVER_HEADING_POINT == m_overWhat) {
                    overControlPoint.setHeadingLocation(pt);
                }
            }
            repaint();
        }

        @Override
        public void mouseMoved(MouseEvent e) {
            Point2D pt = m_mouse = (Point2D.Double) m_mouseXfm.transform(
                    new Point2D.Double(e.getPoint().getX(),e.getPoint().getY()), null);
            if (m_mode == MODE_EDIT) {
                overControlPoint = null;
                m_overWhat = null;
                for (KochanekBartelsSpline.ControlPoint point : path.getControlPoints()) {
                    if (point.testOverControlPoint(pt.getX(), pt.getY(), OVER_TOL/m_scale)) {
                        overControlPoint = point;
                        m_overWhat = OVER_CONTROL_POINT;
                    } else if (point.testOveTangentPoint(pt.getX(), pt.getY(), OVER_TOL/m_scale)) {
                        overControlPoint = point;
                        m_overWhat = OVER_TANGENT_POINT;
                    } else if (point.testOveHeadingPoint(pt.getX(), pt.getY(), OVER_TOL/m_scale)) {
                        overControlPoint = point;
                        m_overWhat = OVER_HEADING_POINT;
                    }
                }
            }
            repaint();
        }
    }

    public PathCanvas(GraphicsConfiguration gc) {
        super(gc);
        // setup all of the stuff for the path panel
        setBackground(Color.BLACK);
        MouseAdapter mouseHandler = new MouseHandler();
        addMouseListener(mouseHandler);
        addMouseMotionListener(mouseHandler);
        ComponentHandler componentHandler = new ComponentHandler();
        addComponentListener(componentHandler);
        // create the robot geometry
        robot = new GeneralPath(GeneralPath.WIND_NON_ZERO, 4);
        robot.moveTo(-ROBOT_WIDTH/2.0, -ROBOT_LENGTH/2.0);
        robot.lineTo(-ROBOT_WIDTH/2.0, ROBOT_LENGTH/2.0);
        robot.lineTo(ROBOT_WIDTH/2.0, ROBOT_LENGTH/2.0);
        robot.lineTo(ROBOT_WIDTH/2.0, -ROBOT_LENGTH/2.0);
        robot.closePath();

        robotBumpers = new GeneralPath(GeneralPath.WIND_NON_ZERO, 4);
        robotBumpers.moveTo(-ROBOT_WIDTH_BUMPER/2.0, -ROBOT_LENGTH_BUMPER/2.0);
        robotBumpers.lineTo(-ROBOT_WIDTH_BUMPER/2.0, ROBOT_LENGTH_BUMPER/2.0);
        robotBumpers.lineTo(ROBOT_WIDTH_BUMPER/2.0, ROBOT_LENGTH_BUMPER/2.0);
        robotBumpers.lineTo(ROBOT_WIDTH_BUMPER/2.0, -ROBOT_LENGTH_BUMPER/2.0);
        robotBumpers.closePath();
    }

    @Override
    public void paint(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setPaint(Color.WHITE);

        // draw the field first, everything else is on top of the field
        m_field.draw(g2d,m_drawXfm);

        // draw the robot at the control points. otherwise, the robot obscures the path and
        // other control point editing handles.
        for (KochanekBartelsSpline.ControlPoint point : path.getControlPoints()) {
            paintRobot(g2d, point);
        }
        g2d.setPaint(Color.WHITE);

        // Draw the path as a set of segments uniformly spaced in time.
        KochanekBartelsSpline.PathPoint lastPathPoint = null;
        Point2D.Double lastPt = null;
        KochanekBartelsSpline.PathPoint thisPathPoint = null;
        Point2D.Double thisPt = null;
        for (KochanekBartelsSpline.PathPoint pathPoint : path.getCurveSegments()) {
            lastPathPoint = thisPathPoint;
            lastPt = thisPt;
            thisPathPoint = pathPoint;
            thisPt = (Point2D.Double)m_drawXfm.transform(pathPoint.fieldPt,null);
            if (lastPathPoint != null) {
                g2d.drawLine((int) lastPt.getX(), (int) lastPt.getY(),
                        (int) thisPt.getX(), (int) thisPt.getY());
            }
            g2d.drawOval((int) thisPt.getX() - 2, (int) thisPt.getY() - 2, 4, 4);
        }

        // draw the control point editing handles.
        for (KochanekBartelsSpline.ControlPoint point : path.getControlPoints()) {
            g2d.setPaint(Color.RED);
            Point2D.Double fieldPt = (Point2D.Double)m_drawXfm.transform(
                    new Point2D.Double(point.m_fieldX, point.m_fieldY),null);
            Point2D.Double tangentPt = (Point2D.Double)m_drawXfm.transform(
                    new Point2D.Double(point.getTangentX(), point.getTangentY()),null);
            g2d.drawOval((int) fieldPt.getX() - 3, (int) fieldPt.getY() - 3, 6, 6);
            g2d.fillOval((int) fieldPt.getX() - 3, (int) fieldPt.getY() - 3, 6, 6);
            g2d.drawLine((int) fieldPt.getX(), (int) fieldPt.getY(), (int)tangentPt.getX(), (int)tangentPt.getY());
            g2d.drawOval((int)tangentPt.getX() - 3, (int)tangentPt.getY() - 3, 6, 6);
            Point2D.Double headingPt = (Point2D.Double)m_drawXfm.transform(
                    new Point2D.Double(point.getHeadingX(), point.getHeadingY()),null);
            g2d.setPaint(Color.MAGENTA);
            g2d.drawLine((int) fieldPt.getX(), (int) fieldPt.getY(), (int)headingPt.getX(), (int)headingPt.getY());
            g2d.drawOval((int)headingPt.getX() - 3, (int)headingPt.getY() - 3, 6, 6);
        }

        // If the cursor is over a control point, highlight it
        if (null != overControlPoint) {
            Stroke oldStroke = g2d.getStroke();
            g2d.setStroke(m_highlightStroke);
            g2d.setPaint(Color.GREEN);
            if (OVER_CONTROL_POINT == m_overWhat) {
                Point2D.Double fieldPt = (Point2D.Double)m_drawXfm.transform(
                        new Point2D.Double(overControlPoint.m_fieldX, overControlPoint.m_fieldY),null);
                g2d.drawOval((int)fieldPt.getX() - 4, (int) fieldPt.getY() - 4,
                        8, 8);
            } else if (OVER_TANGENT_POINT == m_overWhat) {
                Point2D.Double tangentPt = (Point2D.Double)m_drawXfm.transform(
                        new Point2D.Double(overControlPoint.getTangentX(), overControlPoint.getTangentY()),null);
                g2d.drawOval((int)tangentPt.getX() - 4, (int)tangentPt.getY() - 4, 8, 8);
            } else if (OVER_HEADING_POINT == m_overWhat) {
                Point2D.Double headingPt = (Point2D.Double)m_drawXfm.transform(
                        new Point2D.Double(overControlPoint.getHeadingX(), overControlPoint.getHeadingY()),null);
                g2d.drawOval((int)headingPt.getX() - 4, (int)headingPt.getY() - 4, 8, 8);
            }
            g2d.setStroke(oldStroke);
        }

        // draw the mouse and tracking info
        // TODO: handle repositioning the text when the cursor gets to the edge of
        //  the window.
        if (null != m_mouse) {
            g2d.setPaint(Color.WHITE);
            Point2D screenMouse = m_drawXfm.transform(m_mouse,null);
            g2d.drawString(
                    String.format("(%.4f,%.4f)", m_mouse.getX(), m_mouse.getY()),
                    (int) screenMouse.getX(), (int) screenMouse.getY());
        }

    }

    void paintRobot(Graphics2D g2d, KochanekBartelsSpline.ControlPoint controlPoint) {
        paintRobot(g2d,new Point2D.Double(controlPoint.m_fieldX, controlPoint.m_fieldY),
                controlPoint.m_fieldHeading);

    }
    void paintRobot(Graphics2D g2d, KochanekBartelsSpline.PathPoint pathPoint) {


    }

    void paintRobot(Graphics2D g2d, Point2D fieldPt, double heading) {
        AffineTransform oldXfm = g2d.getTransform();
        AffineTransform xfm = new AffineTransform(oldXfm);
        xfm.concatenate(m_drawXfm);
        xfm.translate(fieldPt.getX(), fieldPt.getY());
        xfm.rotate(heading);
        xfm.scale(0.5,0.5);
//        xfm.concatenate(m_drawXfm);
        g2d.setTransform(xfm);
        g2d.setPaint(Color.MAGENTA);
        g2d.draw(robotBumpers);
        g2d.setPaint(Color.BLACK);
        g2d.draw(robot);
        g2d.setTransform(oldXfm);
    }

    public void clearPath() {
        path = new KochanekBartelsSpline();
        newControlPoint = null;
        overControlPoint = null;
        m_mode = MODE_ADD;
        m_overWhat = null;
        repaint();
    }
}
