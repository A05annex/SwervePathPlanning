package frc6831.planner;

import org.a05annex.util.geo2d.KochanekBartelsSpline;
import org.jetbrains.annotations.NotNull;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.GeneralPath;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.geom.Point2D;
import java.io.File;

/**
 * This is the canvas we draw the field and path to. It is derived from a
 * {@link Canvas} so it has all the mechanics to support resizing, mouse interaction,
 * and 2D drawing.
 */
public class PathCanvas extends Canvas implements ActionListener {

    // constants to manage the interaction
    private static final double OVER_TOL = 5.0;

    // The editing mode
    private static final int MODE_ADD = 0;      // adding control points to extend the path
    private static final int MODE_EDIT = 1;     // editing the points that re there

    private static final int OVER_NOTHING = 0;
    private static final int OVER_CONTROL_POINT = 1;
    private static final int OVER_TANGENT_POINT = 2;
    private static final int OVER_HEADING_POINT = 3;
    private static final int OVER_PATH_POINT = 4;

    private final PopupMenu m_contextMenu;
    private final MenuItem m_menuItemClearPath;
    private final MenuItem m_menuItemAnimatePath;
    private final MenuItem m_menuItemExtendPath;
    private final MenuItem m_menuItemEndPath;
    private final MenuItem m_menuItemInsert;
    private final MenuItem m_menuItemDelete;
    private final MenuItem m_menuItemResetTangent;
    private final MenuItem m_menuItemSetTime;
    private final MenuItem m_menuItemInfo;

    // the back buffer to support double buffering
    private int bufferWidth;
    private int bufferHeight;
    private Image bufferImage;
    private Graphics bufferGraphics;

    // the actual data for the robot, field, and path
    private final Robot m_robot;                        // the robot description
    private final Field m_field;
    private final TitleChangeListener m_titleChange;
    private File m_pathFile = null;
    private boolean m_modifiedSinceSave = false;
    private final KochanekBartelsSpline m_path = new KochanekBartelsSpline();
    private AffineTransform m_drawXfm = null;
    private AffineTransform m_mouseXfm = null;
    private double m_scale;

    // members to support robot draw and hit-testing
    private GeneralPath m_robotChassis;
    private GeneralPath m_robotBumpers;
    private Point2D.Double m_robotCorners[] =
            {new Point2D.Double(), new Point2D.Double(), new Point2D.Double(), new Point2D.Double()};
    private Point2D.Double m_xfmRobotCorners[] =
            {new Point2D.Double(), new Point2D.Double(), new Point2D.Double(), new Point2D.Double()};

    // controlling the user interaction with the path
    private KochanekBartelsSpline.ControlPoint m_newControlPoint = null;
    private KochanekBartelsSpline.ControlPoint m_overControlPoint = null;
    private KochanekBartelsSpline.PathPoint m_overPathPoint = null;
    private int m_mode = MODE_ADD;
    private int m_overWhat = OVER_NOTHING;
    private final Stroke m_highlightStroke = new BasicStroke(2.0f);
    private Point2D.Double m_mouse = null;

    // The members that support the path animation functionality
    private Timer m_timer = null;
    private long m_pathStartTime = -1;
    private Double m_currentPathTime = 0.0;
    private KochanekBartelsSpline.PathPoint m_currentPathPoint = null;
    private KochanekBartelsSpline.PathFollower m_pathFollower = null;
    private boolean m_animate = false;


    /**
     * This is the handler for resizing. The main thing in resizing is that we scale the
     * drawing so the field fills the window but maintains the correct field aspect ratio.
     */
    private class ComponentHandler extends ComponentAdapter {
        public void componentResized(ComponentEvent e) {
            Component comp = e.getComponent();
            float width = comp.getWidth();
            float height = comp.getHeight();
            System.out.printf("Size Changed %d,%d%n", (int) width, (int) height);
            // OK, so here we pick whether we scale X or Y to fill the window, reverse Y,
            // and, translate 0,0 to center screen.
            Field.MinMax fieldMinMax = m_field.getMinMax();
            // this is the scale that fits the X into the window
            double scaleX = width / (fieldMinMax.m_maxX - fieldMinMax.m_minX);
            // this is the scale that fits y into the window
            double scaleY = height / (fieldMinMax.m_maxY - fieldMinMax.m_minY);
            // this is the scale that fits them both into the window
            m_scale = Math.min(scaleX, scaleY);
            // OK, what is happening here?? Magic - well, not really. The width/2.0 and height/2.0 bits of the
            // m02 and m12 shift the origin to the center of the screen window. For the default competition field
            // this is great because we adopted 0,0 as center field. For the 2021 at home field, the 0.0 is at
            // a corner of the field - the next term is the shift of the 0,0 for the field from center window,
            // scaled by the field to window scale.
            m_drawXfm = new AffineTransform(m_scale, 0.0f, 0.0f, -m_scale,
                    (width / 2.0) - (m_scale * (((fieldMinMax.m_maxX - fieldMinMax.m_minX) / 2.0) + fieldMinMax.m_minX)),
                    (height / 2.0) + (m_scale * (((fieldMinMax.m_maxY - fieldMinMax.m_minY) / 2.0) + fieldMinMax.m_minY)));
            m_mouseXfm = new AffineTransform(m_drawXfm);
            try {
                m_mouseXfm.invert();
            } catch (NoninvertibleTransformException ex) {
                System.out.println("  -- can't invert draw transform");
            }
        }

    }

    /**
     * This is the handler for mouse actions on the path planning canvas.
     */
    private class MouseHandler extends MouseAdapter {
        @Override
        public void mousePressed(MouseEvent e) {
            Point2D pt = m_mouse = (Point2D.Double) m_mouseXfm.transform(
                    new Point2D.Double(e.getPoint().getX(), e.getPoint().getY()), null);
            if (e.isPopupTrigger()) {
                if (m_mode == MODE_EDIT) {
                    testMouseOver(pt);
                }
                displayContextMenu(e);
            } else if ((m_mode == MODE_ADD) && (e.getClickCount() == 1)) {
                m_newControlPoint = m_path.addControlPoint(pt);
                m_modifiedSinceSave = true;
                repaint();
            } else if (m_mode == MODE_EDIT) {
                testMouseOver(pt);
                repaint();
            }
        }

        @Override
        public void mouseReleased(MouseEvent e) {
            if (e.isPopupTrigger()) {
                displayContextMenu(e);
            } else if (m_mode == MODE_ADD) {
                if (e.getClickCount() == 1) {
                    m_newControlPoint = null;
                    repaint();
                } else if (e.getClickCount() == 2) {
                    setEditMode();
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
                    new Point2D.Double(e.getPoint().getX(), e.getPoint().getY()), null);
            if ((m_mode == MODE_ADD) && null != m_newControlPoint) {
                m_newControlPoint.setFieldLocation(pt);
                m_modifiedSinceSave = true;
            } else if ((m_mode == MODE_EDIT) && (null != m_overControlPoint)) {
                if (OVER_CONTROL_POINT == m_overWhat) {
                    m_overControlPoint.setFieldLocation(pt);
                    m_modifiedSinceSave = true;
                } else if (OVER_TANGENT_POINT == m_overWhat) {
                    m_overControlPoint.setTangentLocation(pt);
                    m_modifiedSinceSave = true;
                } else if (OVER_HEADING_POINT == m_overWhat) {
                    m_overControlPoint.setHeadingLocation(pt);
                    m_modifiedSinceSave = true;
                }
            }
            repaint();
        }

        @Override
        public void mouseMoved(MouseEvent e) {
            Point2D pt = m_mouse = (Point2D.Double) m_mouseXfm.transform(
                    new Point2D.Double(e.getPoint().getX(), e.getPoint().getY()), null);
            if (m_mode == MODE_EDIT) {
                testMouseOver(pt);
            }
            repaint();
        }

        /**
         * Determine whether the mouse is over anything that is significant for path editing.
         *
         * @param pt (Point2D, not null) The position of the mouse transformed to field coordinates.
         */
        private void testMouseOver(@NotNull Point2D pt) {
            m_overControlPoint = null;
            m_overPathPoint = null;
            m_overWhat = OVER_NOTHING;
            for (KochanekBartelsSpline.ControlPoint point : m_path.getControlPoints()) {
                if (point.testOveTangentPoint(pt.getX(), pt.getY(), OVER_TOL / m_scale)) {
                    m_overControlPoint = point;
                    m_overWhat = OVER_TANGENT_POINT;
                } else if (point.testOverControlPoint(pt.getX(), pt.getY(), OVER_TOL / m_scale)) {
                    m_overControlPoint = point;
                    m_overWhat = OVER_CONTROL_POINT;

                } else if (point.testOverHeadingPoint(pt.getX(), pt.getY(), OVER_TOL / m_scale)) {
                    m_overControlPoint = point;
                    m_overWhat = OVER_HEADING_POINT;
                }
            }
            if (OVER_NOTHING == m_overWhat) {
                // Draw the path as a set of segments uniformly spaced in time.
                for (KochanekBartelsSpline.PathPoint pathPoint : m_path.getCurveSegments()) {
                    if (pathPoint.testOverPathPoint(pt.getX(), pt.getY(), OVER_TOL / m_scale)) {
                        m_overPathPoint = pathPoint;
                        m_overWhat = OVER_PATH_POINT;
                        break;
                    }
                }
            }
        }

        private void displayContextMenu(MouseEvent e) {
            // check the current state of the path and editing state and enable choices that are valid in the
            // the current state; disable choices that are invalid in the current state.
            m_menuItemExtendPath.setEnabled(m_mode != MODE_ADD);
            m_menuItemEndPath.setEnabled(m_mode == MODE_ADD);

            boolean controlPointSelected = (null != m_overControlPoint) && (m_overWhat == OVER_CONTROL_POINT);
            m_menuItemDelete.setEnabled(controlPointSelected);
            m_menuItemResetTangent.setEnabled(controlPointSelected && m_overControlPoint.getDerivativesManuallyEdited());
            m_menuItemSetTime.setEnabled(false);
            m_menuItemInfo.setEnabled(false);

            m_menuItemInsert.setEnabled((null != m_overPathPoint) && (m_overWhat == OVER_PATH_POINT));

            m_contextMenu.show(e.getComponent(), e.getX(), e.getY());
        }
    }

    static private MenuItem createMenuItem(PopupMenu menu, String name, ActionListener actionListener) {
        MenuItem menuItem = new MenuItem(name);
        menuItem.addActionListener(actionListener);
        menu.add(menuItem);
        return menuItem;
    }

    /**
     * The constructor for the PathCanvas.
     *
     * @param gc    The graphics configuration that will host this panel
     * @param robot The representation of the robot.
     * @param field The representation of the field.
     */
    public PathCanvas(@NotNull GraphicsConfiguration gc, @NotNull Robot robot,
                      @NotNull Field field, @NotNull TitleChangeListener titleChange) {
        super(gc);
        m_robot = robot;
        m_field = field;
        m_titleChange = titleChange;

        // build the right menu popup
        m_contextMenu = new PopupMenu();
        m_menuItemClearPath = createMenuItem(m_contextMenu, "Clear Path", this);
        m_menuItemAnimatePath = createMenuItem(m_contextMenu, "Play Path", this);
        m_menuItemExtendPath = createMenuItem(m_contextMenu, "Extend Path", this);
        m_menuItemEndPath = createMenuItem(m_contextMenu, "End Path", this);
        m_contextMenu.addSeparator();
        m_menuItemInsert = createMenuItem(m_contextMenu, "Insert Control Point", this);
        m_menuItemDelete = createMenuItem(m_contextMenu, "Delete Control Point", this);
        m_menuItemResetTangent = createMenuItem(m_contextMenu, "Reset Tangent", this);
        m_menuItemSetTime = createMenuItem(m_contextMenu, "Set Time", this);
        m_menuItemInfo = createMenuItem(m_contextMenu, "Info", this);
        add(m_contextMenu);

        // setup all of the stuff for the path panel
        setBackground(Color.BLACK);
        MouseAdapter mouseHandler = new MouseHandler();
        addMouseListener(mouseHandler);
        addMouseMotionListener(mouseHandler);
        addComponentListener(new ComponentHandler());
        resetRobotGeometry();
    }

    /**
     * This should be called if a new representation of the robot is loaded so that a new graphic
     * representation of the robot can be created.
     */
    public void resetRobotGeometry() {
        // create the robot geometry
        m_robotChassis = new GeneralPath(GeneralPath.WIND_NON_ZERO, 4);
        m_robotChassis.moveTo(-m_robot.getChassisWidth() / 2.0, -m_robot.getChassisLength() / 2.0);
        m_robotChassis.lineTo(-m_robot.getChassisWidth() / 2.0, m_robot.getChassisLength() / 2.0);
        m_robotChassis.lineTo(m_robot.getChassisWidth() / 2.0, m_robot.getChassisLength() / 2.0);
        m_robotChassis.lineTo(m_robot.getChassisWidth() / 2.0, -m_robot.getChassisLength() / 2.0);
        m_robotChassis.closePath();

        m_robotBumpers = new GeneralPath(GeneralPath.WIND_NON_ZERO, 4);
        m_robotBumpers.moveTo(-m_robot.getBumperWidth() / 2.0, -m_robot.getBumperLength() / 2.0);
        m_robotBumpers.lineTo(-m_robot.getBumperWidth() / 2.0, m_robot.getBumperLength() / 2.0);
        m_robotBumpers.lineTo(m_robot.getBumperWidth() / 2.0, m_robot.getBumperLength() / 2.0);
        m_robotBumpers.lineTo(m_robot.getBumperWidth() / 2.0, -m_robot.getBumperLength() / 2.0);
        m_robotBumpers.closePath();

        m_robotCorners[0].x = -m_robot.getBumperWidth() / 2.0;
        m_robotCorners[0].y = -m_robot.getBumperLength() / 2.0;
        m_robotCorners[1].x = -m_robot.getBumperWidth() / 2.0;
        m_robotCorners[1].y = m_robot.getBumperLength() / 2.0;
        m_robotCorners[2].x = m_robot.getBumperWidth() / 2.0;
        m_robotCorners[2].y = m_robot.getBumperLength() / 2.0;
        m_robotCorners[3].x = m_robot.getBumperWidth() / 2.0;
        m_robotCorners[3].y = -m_robot.getBumperLength() / 2.0;

    }

    /**
     * An action listener for the {@link PathCanvas} that is specifically looking for {@link #m_timer} events
     * during path animation.
     *
     * @param event (ActionEvent) The action event that was sent to this path canvas.
     */
    @Override
    public void actionPerformed(ActionEvent event) {
        final Object src = event.getSource();
        if ((event.getSource() == m_timer) && (null != m_pathFollower)) {
            // This is a timer event while there is a path follower.
            if (m_pathStartTime == -1) {
                //  This is the start of the animation. The robot is currently drawn at the start position
                // so the only action is to record the start time.
                m_pathStartTime = event.getWhen();
            } else {
                // This is a point at some time on the path
                m_currentPathTime = (event.getWhen() - m_pathStartTime) / 1000.0;
                m_currentPathPoint = m_pathFollower.getPointAt(m_currentPathTime);
                if (null == m_currentPathPoint) {
                    stopAnimation();
                    // reached the end of the path
                }
                repaint();
            }
        } else if (src == m_menuItemClearPath) {
            clearPath();
        } else if (src == m_menuItemAnimatePath) {
            animatePath();
        } else if (src == m_menuItemEndPath) {
            setEditMode();
        } else if (src == m_menuItemExtendPath) {
            setExtendMode();
        } else if (src == m_menuItemInsert) {
            m_path.insertControlPointBefore(m_overPathPoint.nextControlPoint,
                    m_overPathPoint.fieldPt.getX(), m_overPathPoint.fieldPt.getY(),
                    m_overPathPoint.fieldHeading);
            m_modifiedSinceSave = true;
            repaint();
        } else if (src == m_menuItemDelete) {
            m_path.deleteControlPoint(m_overControlPoint);
            m_overControlPoint = null;
            m_overWhat = OVER_NOTHING;
            m_modifiedSinceSave = true;
            repaint();
        } else if (src == m_menuItemResetTangent) {
            m_overControlPoint.resetDerivative();
            m_modifiedSinceSave = true;
            repaint();
        } else if (src == m_menuItemSetTime) {

        } else if (src == m_menuItemInfo) {

        }
    }

    /**
     * This override does not do anything other than call {@link #paint(Graphics)}. The overridden method assumed
     * the update was drawing to the displayed video buffer, so it cleared the buffer and then drew the new content,
     * which results in a lot of screen flashing for older video cards.
     *
     * @param g Thr graphic context to be updated (repainted)
     */
    @Override
    public void update(Graphics g) {
        paint(g);
    }

    /**
     * Paint the panel, which it the double buffer context means:
     * <ul>
     *     <li>make sure there is a back buffer that is the size of the panel.</li>
     *     <li>clear the back buffer</li>
     *     <li>paint the current content into the back buffer</li>
     *     <li>copy the back buffer to the panel</li>
     * </ul>
     *
     * @param g The graphics context for drawing to tha back buffer.
     */
    @Override
    public void paint(Graphics g) {
        // make sure there is a back buffer that is the size of the onscreen panel
        if (bufferWidth != getSize().width ||
                bufferHeight != getSize().height ||
                bufferImage == null || bufferGraphics == null) {
            resetBuffer();
        }
        if (bufferGraphics != null) {
            //this clears the back buffer
            bufferGraphics.clearRect(0, 0, bufferWidth, bufferHeight);

            // draw the content to the back buffer
            paintBuffer(bufferGraphics);

            // copy the back buffer into this displayed panel
            g.drawImage(bufferImage, 0, 0, this);
        }
    }

    /**
     * Create a back buffer that is the size of the panel.
     */
    private void resetBuffer() {
        // always keep track of the image size
        bufferWidth = getSize().width;
        bufferHeight = getSize().height;

        //    clean up the previous image
        if (bufferGraphics != null) {
            bufferGraphics.dispose();
            bufferGraphics = null;
        }
        if (bufferImage != null) {
            bufferImage.flush();
            bufferImage = null;
        }

        //    create the new image with the size of the panel
        bufferImage = createImage(bufferWidth, bufferHeight);
        bufferGraphics = bufferImage.getGraphics();
    }

    /**
     * Paint the current field, robot, and path to the back buffer.
     *
     * @param g The graphics description for the back buffer.
     */
    public void paintBuffer(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setPaint(Color.WHITE);

        // draw the field first, everything else is on top of the field
        m_field.draw(g2d, m_drawXfm);

        // draw the robot at the control points. otherwise, the robot obscures the path and
        // other control point editing handles.
        if (m_animate) {
            g2d.drawString(
                    String.format("time = %.3f", m_currentPathTime), 10, 20);
            g2d.drawString(
                    String.format("forward = %.3f", m_currentPathPoint.speedForward), 10, 35);
            g2d.drawString(
                    String.format("strafe = %.3f", m_currentPathPoint.speedStrafe), 10, 50);
            g2d.drawString(
                    String.format("angular vel = %.3f", m_currentPathPoint.speedRotation), 10, 65);
            boolean tooFast = !m_robot.canRobotAchieve(m_currentPathPoint.speedForward,
                    m_currentPathPoint.speedStrafe,m_currentPathPoint.speedRotation);
            System.out.printf("%10.3f, %10.3f, %10.3f, %10.3f      %b %n",
                    m_currentPathTime, m_currentPathPoint.speedForward,
                    m_currentPathPoint.speedStrafe, m_currentPathPoint.speedRotation, tooFast);
            paintRobot(g2d, m_currentPathPoint, tooFast);
            g2d.setPaint(Color.MAGENTA);
            double fieldX = m_currentPathPoint.fieldPt.getX();
            double fieldY = m_currentPathPoint.fieldPt.getY();
            Point2D.Double fieldPt = (Point2D.Double) m_drawXfm.transform(
                    new Point2D.Double(fieldX, fieldY), null);
            double dirX = fieldX + Math.sin(m_currentPathPoint.fieldHeading);
            double dirY = fieldY + Math.cos(m_currentPathPoint.fieldHeading);
            Point2D.Double dirPt = (Point2D.Double) m_drawXfm.transform(
                    new Point2D.Double(dirX, dirY), null);
            g2d.drawLine((int) fieldPt.getX(), (int) fieldPt.getY(), (int) dirPt.getX(), (int) dirPt.getY());
            g2d.drawOval((int) dirPt.getX() - 3, (int) dirPt.getY() - 3, 6, 6);

        } else {
            for (KochanekBartelsSpline.ControlPoint point : m_path.getControlPoints()) {
                paintRobot(g2d, point);
            }
        }
        g2d.setPaint(Color.WHITE);

        // Draw the path as a set of segments uniformly spaced in time.
        KochanekBartelsSpline.PathPoint lastPathPoint;
        Point2D.Double lastPt;
        KochanekBartelsSpline.PathPoint thisPathPoint = null;
        Point2D.Double thisPt = null;
        for (KochanekBartelsSpline.PathPoint pathPoint : m_path.getCurveSegments()) {
            lastPathPoint = thisPathPoint;
            lastPt = thisPt;
            thisPathPoint = pathPoint;
            boolean tooFast = !m_robot.canRobotAchieve(pathPoint.speedForward,
                    pathPoint.speedStrafe,pathPoint.speedRotation);

            g2d.setPaint(isRobotInside(pathPoint.fieldPt, pathPoint.fieldHeading) ?
                    (tooFast ? Color.RED : Color.WHITE) : Color.ORANGE);
            thisPt = (Point2D.Double) m_drawXfm.transform(pathPoint.fieldPt, null);

            if (lastPathPoint != null) {
                g2d.drawLine((int) lastPt.getX(), (int) lastPt.getY(),
                        (int) thisPt.getX(), (int) thisPt.getY());
            }
            g2d.drawOval((int) thisPt.getX() - 2, (int) thisPt.getY() - 2, 4, 4);
        }

        if (!m_animate) {
            // draw the control point editing handles.
            for (KochanekBartelsSpline.ControlPoint point : m_path.getControlPoints()) {
                g2d.setPaint(Color.RED);
                Point2D.Double fieldPt = (Point2D.Double) m_drawXfm.transform(
                        new Point2D.Double(point.getFieldX(), point.getFieldY()), null);
                Point2D.Double tangentPt = (Point2D.Double) m_drawXfm.transform(
                        new Point2D.Double(point.getTangentX(), point.getTangentY()), null);
                g2d.drawOval((int) fieldPt.getX() - 3, (int) fieldPt.getY() - 3, 6, 6);
                g2d.fillOval((int) fieldPt.getX() - 3, (int) fieldPt.getY() - 3, 6, 6);
                g2d.drawLine((int) fieldPt.getX(), (int) fieldPt.getY(), (int) tangentPt.getX(), (int) tangentPt.getY());
                g2d.drawOval((int) tangentPt.getX() - 3, (int) tangentPt.getY() - 3, 6, 6);
                Point2D.Double headingPt = (Point2D.Double) m_drawXfm.transform(
                        new Point2D.Double(point.getHeadingX(), point.getHeadingY()), null);
                g2d.setPaint(Color.MAGENTA);
                g2d.drawLine((int) fieldPt.getX(), (int) fieldPt.getY(), (int) headingPt.getX(), (int) headingPt.getY());
                g2d.drawOval((int) headingPt.getX() - 3, (int) headingPt.getY() - 3, 6, 6);
            }

            // If the cursor is over a control point or path point, highlight it
            if (m_overWhat != OVER_NOTHING) {
                Stroke oldStroke = g2d.getStroke();
                g2d.setStroke(m_highlightStroke);
                g2d.setPaint(m_overWhat == OVER_PATH_POINT ? Color.ORANGE : Color.GREEN);
                switch (m_overWhat) {
                    case OVER_CONTROL_POINT:
                        drawFieldPointHighlight(g2d, m_overControlPoint.getFieldX(), m_overControlPoint.getFieldY());
                        break;
                    case OVER_TANGENT_POINT:
                        drawFieldPointHighlight(g2d,
                                m_overControlPoint.getTangentX(), m_overControlPoint.getTangentY());
                        break;
                    case OVER_HEADING_POINT:
                        drawFieldPointHighlight(g2d,
                                m_overControlPoint.getHeadingX(), m_overControlPoint.getHeadingY());
                        break;
                    case OVER_PATH_POINT:
                        drawFieldPointHighlight(g2d, m_overPathPoint.fieldPt.getX(), m_overPathPoint.fieldPt.getY());
                        break;

                }
                g2d.setStroke(oldStroke);
            }

            // draw the mouse and tracking info
            // TODO: handle repositioning the text when the cursor gets to the edge of
            //  the window.
            if (null != m_mouse) {
                g2d.setPaint(Color.WHITE);
                Point2D screenMouse = m_drawXfm.transform(m_mouse, null);
                g2d.drawString(
                        String.format("(%.4f,%.4f)", m_mouse.getX(), m_mouse.getY()),
                        (int) screenMouse.getX(), (int) screenMouse.getY());
            }
        }
    }

    private void drawFieldPointHighlight(Graphics2D g2d, double fieldX, double fieldY) {
        Point2D.Double fieldPt = (Point2D.Double) m_drawXfm.transform(
                new Point2D.Double(fieldX, fieldY), null);
        g2d.drawOval((int) fieldPt.getX() - 4, (int) fieldPt.getY() - 4, 8, 8);
    }

    public KochanekBartelsSpline getPath() {
        return m_path;
    }

    private void paintRobot(Graphics2D g2d, KochanekBartelsSpline.ControlPoint controlPoint) {
        paintRobot(g2d, new Point2D.Double(controlPoint.getFieldX(), controlPoint.getFieldY()),
                controlPoint.getFieldHeading(), false);

    }

    private void paintRobot(Graphics2D g2d, KochanekBartelsSpline.PathPoint pathPoint, boolean tooFast) {
        paintRobot(g2d, pathPoint.fieldPt, pathPoint.fieldHeading, tooFast);


    }

    private void paintRobot(Graphics2D g2d, Point2D fieldPt, double heading, boolean tooFast) {
        AffineTransform oldXfm = g2d.getTransform();
        AffineTransform xfm = new AffineTransform(oldXfm);
        xfm.concatenate(m_drawXfm);

        AffineTransform xfmRobot = new AffineTransform();
        xfmRobot.translate(fieldPt.getX(), fieldPt.getY());
        xfmRobot.rotate(-heading);
        xfm.concatenate(xfmRobot);
        // don't know why this scale is required, it should be on the oldXfm or the field rendering
        //  would be wrong ---- TODO. figure this out.
        xfm.scale(0.5, 0.5);

        xfmRobot.transform(m_robotCorners, 0, m_xfmRobotCorners, 0, 4);
        boolean inside = m_field.isInsideField(m_xfmRobotCorners, 0.05);

        g2d.setTransform(xfm);
        g2d.setPaint(inside ? (tooFast ? Color.RED : Color.MAGENTA) : Color.ORANGE);
        g2d.draw(m_robotBumpers);
        g2d.fill(m_robotBumpers);
        g2d.setPaint(Color.BLACK);
        g2d.draw(m_robotChassis);
        g2d.fill(m_robotChassis);
        g2d.setTransform(oldXfm);
    }

    private boolean isRobotInside(Point2D fieldPt, double heading) {
        AffineTransform xfmRobot = new AffineTransform();
        xfmRobot.translate(fieldPt.getX(), fieldPt.getY());
        xfmRobot.rotate(-heading);
        xfmRobot.transform(m_robotCorners, 0, m_xfmRobotCorners, 0, 4);
        return m_field.isInsideField(m_xfmRobotCorners, 0.05);
    }

    public void setEditMode() {
        m_newControlPoint = null;
        m_overControlPoint = null;
        m_mode = MODE_EDIT;
        m_overWhat = OVER_NOTHING;
    }

    public void setExtendMode() {
        m_newControlPoint = null;
        m_overControlPoint = null;
        m_mode = MODE_ADD;
        m_overWhat = OVER_NOTHING;
    }

    /**
     * Get the name of the path filename.
     *
     * @return {@code null} if a filename has not been set, otherwise the name of the current
     * path file.
     */
    public File getPathFile() {
        return m_pathFile;
    }

    /**
     * Test whether the path been modified since the last save.
     *
     * @return {@code true} the last changes have not been saved, {@code false} if there have
     * been no path changes since the last path save.
     */
    public boolean modifiedSinceSave() {
        return m_modifiedSinceSave;
    }

    /**
     * Start a new path. This means clear the current path (all the control points, etc.) and
     * restart creating a path. This is different from clearing a path which maintains the context
     * of the path.
     */
    public void newPath() {
        clearPath();
        m_pathFile = null;
        m_modifiedSinceSave = false;
        m_titleChange.titleChanged();
    }

    /**
     * Clear the current path and restart drawing that path.
     */
    public void clearPath() {
        m_path.clearPath();
        setExtendMode();
        m_pathFile = null;
        m_modifiedSinceSave = true;
        repaint();
    }

    /**
     * The menu action to load a path. This displays a file chooser, reads the path and saves the
     * absolute directory path to the path file.
     */
    public void loadPath() {
        JFileChooser fc = new JFileChooser(System.getProperty("user.dir"));
        fc.setDialogTitle("Load Path");
        fc.setFileFilter(new FileNameExtensionFilter("JSON file", "json"));
        fc.setAcceptAllFileFilterUsed(false);
        if (JFileChooser.APPROVE_OPTION == fc.showOpenDialog(this)) {
            File file = fc.getSelectedFile();
            System.out.println("Loading path from: " + file.getAbsolutePath());
            m_pathFile = file;
            m_path.loadPath(file.getAbsolutePath());
            m_modifiedSinceSave = false;
            m_titleChange.titleChanged();
        } else {
            System.out.println("Load path command cancelled by user.");
        }
        setEditMode();
        repaint();
    }

    /**
     * The menu action to reload a path. Very useful if you are hand editing a path.
     */
    public void reloadPath() {
        System.out.println("Reloading path from: " + m_pathFile.getAbsolutePath());
        m_path.loadPath(m_pathFile.getAbsolutePath());
        m_modifiedSinceSave = false;
        setEditMode();
        repaint();
    }

    /**
     * Save the path file to the current path file.
     */
    public void savePath() {
        System.out.println("Saving path as: " + m_pathFile.getAbsolutePath());
        m_path.savePath(m_pathFile.getAbsolutePath());
        m_modifiedSinceSave = false;
        setEditMode();
        repaint();
    }

    /**
     * Save the path to a new path file.
     */
    public void savePathAs() {
        JFileChooser fc = new JFileChooser(System.getProperty("user.dir"));
        fc.setDialogTitle("Save Path As");
        fc.setFileFilter(new FileNameExtensionFilter("JSON file", "json"));
        fc.setAcceptAllFileFilterUsed(false);
        if (JFileChooser.APPROVE_OPTION == fc.showSaveDialog(this)) {
            m_pathFile = fc.getSelectedFile();
            if (!m_pathFile.getAbsolutePath().endsWith(".json")) {
                m_pathFile = new File(m_pathFile.getAbsolutePath() + ".json");
            }
            savePath();
            m_titleChange.titleChanged();
        } else {
            System.out.println("Save path command cancelled by user.");
        }
    }

    /**
     * Animate the robot position on the path from start to end of the path.
     */
    public void animatePath() {
        if (null == m_timer) {
            // create the timer if one does not exist
            m_timer = new Timer(20, this);
            m_timer.setInitialDelay(200);
            m_timer.start();
        } else {
            m_timer.restart();
        }
        m_animate = true;
        m_pathStartTime = -1;
        m_currentPathTime = 0.0;
        m_pathFollower = m_path.getPathFollower();
        m_currentPathPoint = m_pathFollower.getPointAt(m_currentPathTime);
        System.out.printf("    seconds     forward      strafe     angular    too fast!%n");
        if (null == m_currentPathPoint) {
            stopAnimation();
        }
        repaint();
    }

    public void stopAnimation() {
        m_timer.stop();
        m_animate = false;
        m_pathFollower = null;
        m_currentPathTime = 0.0;
        m_pathStartTime = -1;
    }
}
