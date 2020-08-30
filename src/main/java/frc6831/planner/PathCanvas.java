package frc6831.planner;

import frc6831.lib2d.KochanekBartelsSpline;

import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;

public class PathCanvas extends Canvas {

    private KochanekBartelsSpline path = new KochanekBartelsSpline();
    private KochanekBartelsSpline.ControlPoint newControlPoint = null;

    private class MouseHandler extends MouseAdapter {
        @Override
        public void mousePressed(MouseEvent e) {
            newControlPoint = path.addControlPoint(e.getPoint());
            repaint();
        }

        @Override
        public void mouseReleased(MouseEvent e) {
            newControlPoint = null;
            repaint();
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
            newControlPoint.setFieldLocation(e.getPoint());
            repaint();
        }

        @Override
        public void mouseMoved(MouseEvent e) {
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
            int endX = (int) (point.m_fieldX + point.m_dXin);
            int endY = (int) (point.m_fieldY + point.m_dYin);
            g2d.drawLine((int) point.m_fieldX, (int) point.m_fieldY, endX, endY);
            g2d.drawOval(endX - 3, endY - 3, 6, 6);
        }
    }
}
