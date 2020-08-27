package frc6831.planner;

import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class PathCanvas extends Canvas {
    private class ControlPoint {
        public double fieldX = 0.0;
        public double fieldY = 0.0;
        public double fieldHeading = 0.0;
        public double bias = 0.0;
        public double tension = 0.0;
        public double continuity = 0.0;

        public ControlPoint() {
        }

        public ControlPoint(Point2D pt) {
            this.fieldX = pt.getX();
            this.fieldY = pt.getY();
        }

        public ControlPoint(double fieldX, double fieldY) {
            this.fieldX = fieldX;
            this.fieldY = fieldY;
        }

        public ControlPoint(double fieldX, double fieldY, double fieldHeading) {
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.fieldHeading = fieldHeading;
        }

        void setFieldLocation(Point2D pt) {
            fieldX = pt.getX();
            fieldY = pt.getY();
        }
    }

    private List<ControlPoint> controlPoints = new ArrayList<>();
    private ControlPoint newControlPoint = null;

    private class MouseHandler extends MouseAdapter {
        @Override
        public void mousePressed(MouseEvent e) {
            newControlPoint = new ControlPoint(e.getPoint());
            controlPoints.add(newControlPoint);
            repaint();
        }

        @Override
        public void mouseReleased(MouseEvent e) {
            newControlPoint = null;
            repaint();
        }

        @Override
        public void mouseEntered(MouseEvent e) {}

        @Override
        public void mouseExited(MouseEvent e) {}

        @Override
        public void mouseWheelMoved(MouseWheelEvent e){}

        @Override
        public void mouseDragged(MouseEvent e){
            newControlPoint.setFieldLocation(e.getPoint());
            repaint();
        }

        @Override
        public void mouseMoved(MouseEvent e){}
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
        Graphics2D g2d = ( Graphics2D ) g;
        g2d.setPaint ( Color.WHITE );
        for (ControlPoint point : controlPoints) {
            g2d.drawOval((int)point.fieldX,(int)point.fieldY,3,3);
        }
    }
}
