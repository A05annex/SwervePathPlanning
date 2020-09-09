package frc6831.planner;

import frc6831.lib2d.KochanekBartelsSpline;

import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.awt.geom.Point2D;

public class PathCanvas extends Canvas {

    private static final double OVER_TOL = 3.0;
    private static final String MODE_ADD = "add";
    private static final String MODE_EDIT = "edit";
    private static final String OVER_CONTROL_POINT = "controlPoint";
    private static final String OVER_TANGENT_POINT = "tangentPoint";
    private KochanekBartelsSpline path = new KochanekBartelsSpline();
    private KochanekBartelsSpline.ControlPoint newControlPoint = null;
    private KochanekBartelsSpline.ControlPoint overControlPoint = null;
    private String m_mode = MODE_ADD;
    private String m_overWhat = null;
    private Stroke m_highlightStroke = new BasicStroke(2.0f);

    private class MouseHandler extends MouseAdapter {
        @Override
        public void mousePressed(MouseEvent e) {
            if ((m_mode == MODE_ADD) && (e.getClickCount() == 1)) {
                newControlPoint = path.addControlPoint(e.getPoint());
                repaint();
            } else if (m_mode == MODE_EDIT) {
                Point2D pt = e.getPoint();
                overControlPoint = null;
                m_overWhat = null;
                for (KochanekBartelsSpline.ControlPoint point : path.getControlPoints()) {
                    if (point.testOverControlPoint(pt.getX(), pt.getY(), OVER_TOL)) {
                        overControlPoint = point;
                        m_overWhat = OVER_CONTROL_POINT;
                    } else if (point.testOveTangentPoint(pt.getX(), pt.getY(), OVER_TOL)) {
                        overControlPoint = point;
                        m_overWhat = OVER_TANGENT_POINT;
                    }
                }
                repaint();
            }
        }

        @Override
        public void mouseReleased(MouseEvent e) {
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
            if (m_mode == MODE_ADD) {
                newControlPoint.setFieldLocation(e.getPoint());
                repaint();
            } else if ((m_mode == MODE_EDIT) && (null != overControlPoint)) {
                if (OVER_CONTROL_POINT == m_overWhat) {
                    overControlPoint.setFieldLocation(e.getPoint());
                    repaint();
                } else if (OVER_TANGENT_POINT == m_overWhat) {
                    overControlPoint.setTangentLocation(e.getPoint());
                    repaint();
                }
            }
        }

        @Override
        public void mouseMoved(MouseEvent e) {
            if (m_mode == MODE_EDIT) {
                Point2D pt = e.getPoint();
                overControlPoint = null;
                m_overWhat = null;
                for (KochanekBartelsSpline.ControlPoint point : path.getControlPoints()) {
                    if (point.testOverControlPoint(pt.getX(), pt.getY(), OVER_TOL)) {
                        overControlPoint = point;
                        m_overWhat = OVER_CONTROL_POINT;
                    } else if (point.testOveTangentPoint(pt.getX(), pt.getY(), OVER_TOL)) {
                        overControlPoint = point;
                        m_overWhat = OVER_TANGENT_POINT;
                    }
                }
                repaint();
            }
        }
    }

    public PathCanvas(GraphicsConfiguration gc) {
        super(gc);
        setBackground(Color.BLACK);
        MouseAdapter mouseHandler = new MouseHandler();
        addMouseListener(mouseHandler);
        addMouseMotionListener(mouseHandler);
    }

    @Override
    public void paint(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setPaint(Color.WHITE);
        KochanekBartelsSpline.PathPoint lastPathPoint = null;
        KochanekBartelsSpline.PathPoint thisPathPoint = null;
        for (KochanekBartelsSpline.PathPoint pathPoint : path.getCurveSegments()) {
            lastPathPoint = thisPathPoint;
            thisPathPoint = pathPoint;
            if (lastPathPoint != null) {
                g2d.drawLine((int) lastPathPoint.fieldX, (int) lastPathPoint.fieldY,
                        (int) thisPathPoint.fieldX, (int) thisPathPoint.fieldY);
            }
            g2d.drawOval((int) pathPoint.fieldX - 2, (int) pathPoint.fieldY - 2, 4, 4);
        }
        g2d.setPaint(Color.RED);
        for (KochanekBartelsSpline.ControlPoint point : path.getControlPoints()) {
            g2d.drawOval((int) point.m_fieldX - 3, (int) point.m_fieldY - 3, 6, 6);
            g2d.fillOval((int) point.m_fieldX - 3, (int) point.m_fieldY - 3, 6, 6);
            int endX = (int) (point.m_fieldX + point.m_dX);
            int endY = (int) (point.m_fieldY + point.m_dY);
            g2d.drawLine((int) point.m_fieldX, (int) point.m_fieldY, (int)point.getTangentX(), (int)point.getTangentY());
            g2d.drawOval((int)point.getTangentX() - 3, (int)point.getTangentY() - 3, 6, 6);
        }
        if (null != overControlPoint) {
            Stroke oldStroke = g2d.getStroke();
            g2d.setStroke(m_highlightStroke);
            g2d.setPaint(Color.GREEN);
            if (OVER_CONTROL_POINT == m_overWhat) {
                g2d.drawOval((int) overControlPoint.m_fieldX - 4, (int) overControlPoint.m_fieldY - 4,
                        8, 8);
            } else if (OVER_TANGENT_POINT == m_overWhat) {
                int endX = (int) (overControlPoint.m_fieldX + overControlPoint.m_dX);
                int endY = (int) (overControlPoint.m_fieldY + overControlPoint.m_dY);
                g2d.drawOval(endX - 4, endY - 4, 8, 8);
            }
            g2d.setStroke(oldStroke);
        }
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
