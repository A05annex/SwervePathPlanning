package frc6831.planner;

import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;
import org.a05annex.util.geo2d.KochanekBartelsSpline;
import org.a05annex.util.geo2d.KochanekBartelsSpline.ControlPoint;
import org.a05annex.util.geo2d.KochanekBartelsSpline.PathPoint;
import org.a05annex.util.geo2d.KochanekBartelsSpline.RobotActionType;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

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

    private final PopupMenu contextMenu;
    private final MenuItem menuItemClearPath;
    private final MenuItem menuSwitchAlliance;
    private final MenuItem menuItemAnimatePath;
    private final MenuItem menuItemStopAnimate;
    private final MenuItem menuItemExtendPath;
    private final MenuItem menuItemEndPath;
    private final MenuItem menuItemInsert;
    private final MenuItem menuItemDelete;
    private final MenuItem menuItemResetTangent;
    private final MenuItem menuItemSetTime;
    private final MenuItem menuItemInfo;

    // the back buffer to support double buffering
    private int bufferWidth;
    private int bufferHeight;
    private Image bufferImage;
    private Graphics bufferGraphics;

    // the actual data for the robot, field, and path
    private final Robot robot;                        // the robot description
    private final Field field;
    private final TitleChangeListener titleChange;
    private File pathFile = null;
    private boolean modifiedSinceSave = false;
    private final KochanekBartelsSpline path = new KochanekBartelsSpline();
    private AffineTransform drawXfm = null;
    private AffineTransform mouseXfm = null;
    private double scale;

    // members to support robot draw and hit-testing
    private GeneralPath robotChassis;
    private GeneralPath robotBumpers;
    private final Point2D.Double[] robotCorners =
            {new Point2D.Double(), new Point2D.Double(), new Point2D.Double(), new Point2D.Double()};
    private final Point2D.Double[] xfmRobotCorners =
            {new Point2D.Double(), new Point2D.Double(), new Point2D.Double(), new Point2D.Double()};

    // controlling the user interaction with the path
    private ControlPoint newControlPoint = null;
    private ControlPoint overControlPoint = null;
    private PathPoint overPathPoint = null;
    private int mode = MODE_ADD;
    private int overWhat = OVER_NOTHING;
    private final Stroke highlightStroke = new BasicStroke(2.0f);
    private Point2D.Double mouse = null;

    // The members that support the path animation functionality
    private Timer timer = null;
    private long pathStartTime = -1;
    private Double currentPathTime = 0.0;
    private long stopAndRunEndTime = -1;
    private String stopAndRunDescription = null;
    private long stopAndRunDuration = 0;
    private PathPoint currentPathPoint = null;
    private KochanekBartelsSpline.PathFollower pathFollower = null;
    private boolean animate = false;

    // The symbol for a stop and run action
    private final int[] robotStopAndRunActionX = {0,  5,  5,  0, -5, -5};
    private final int[] robotStopAndRunActionY = {6,  3, -3, -6, -3,  3};
    // The symbol for schedule action
    private final int[] robotScheduleActionX = {0,  7,  0, -7};
    private final int[] robotScheduleActionY = {7,  0, -7,  0};

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
            Field.MinMax fieldMinMax = field.getMinMax();
            // this is the scale that fits the X into the window
            double scaleX = width / (fieldMinMax.m_maxX - fieldMinMax.m_minX);
            // this is the scale that fits y into the window
            double scaleY = height / (fieldMinMax.m_maxY - fieldMinMax.m_minY);
            // this is the scale that fits them both into the window
            scale = Math.min(scaleX, scaleY);
            // OK, what is happening here?? Magic - well, not really. The width/2.0 and height/2.0 bits of the
            // m02 and m12 shift the origin to the center of the screen window. For the default competition field
            // this is great because we adopted 0,0 as center field. For the 2021 at home field, the 0.0 is at
            // a corner of the field - the next term is the shift of the 0,0 for the field from center window,
            // scaled by the field to window scale.
            drawXfm = new AffineTransform(scale, 0.0f, 0.0f, -scale,
                    (width / 2.0) - (scale * (((fieldMinMax.m_maxX - fieldMinMax.m_minX) / 2.0) + fieldMinMax.m_minX)),
                    (height / 2.0) + (scale * (((fieldMinMax.m_maxY - fieldMinMax.m_minY) / 2.0) + fieldMinMax.m_minY)));
            mouseXfm = new AffineTransform(drawXfm);
            try {
                mouseXfm.invert();
            } catch (NoninvertibleTransformException ex) {
                System.out.println("  -- can't invert draw transform");
            }
        }

    }

    /**
     * This is the handler for mouse actions on the path planning canvas. It handles highlighting
     * what the mouse is over, selection of editable targets (like control point location of tangent
     * and heading handles), and dragging things around on the game canvas.
     */
    private class MouseHandler extends MouseAdapter {
        @Override
        public void mousePressed(MouseEvent e) {
            Point2D pt = mouse = (Point2D.Double) mouseXfm.transform(
                    new Point2D.Double(e.getPoint().getX(), e.getPoint().getY()), null);
            if (e.isPopupTrigger()) {
                // right click - Display a context appropriate popup
                if (mode == MODE_EDIT) {
                    testMouseOver(pt);
                }
                displayContextMenu(e);
            } else if ((mode == MODE_ADD) && (e.getClickCount() == 1)) {
                newControlPoint = path.addControlPoint(pt);
                modifiedSinceSave = true;
                repaint();
            } else if (mode == MODE_EDIT) {
                testMouseOver(pt);
                repaint();
            }
        }

        @Override
        public void mouseReleased(MouseEvent e) {
            if (e.isPopupTrigger()) {
                displayContextMenu(e);
            } else if (mode == MODE_ADD) {
                if (e.getClickCount() == 1) {
                    newControlPoint = null;
                    repaint();
                } else if (e.getClickCount() == 2) {
                    pkgSetEditMode();
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
            Point2D pt = mouse = (Point2D.Double) mouseXfm.transform(
                    new Point2D.Double(e.getPoint().getX(), e.getPoint().getY()), null);
            if ((mode == MODE_ADD) && null != newControlPoint) {
                newControlPoint.setFieldLocation(pt);
                modifiedSinceSave = true;
            } else if ((mode == MODE_EDIT) && (null != overControlPoint)) {
                if (OVER_CONTROL_POINT == overWhat) {
                    overControlPoint.setFieldLocation(pt);
                    modifiedSinceSave = true;
                } else if (OVER_TANGENT_POINT == overWhat) {
                    overControlPoint.setTangentLocation(pt);
                    modifiedSinceSave = true;
                } else if (OVER_HEADING_POINT == overWhat) {
                    overControlPoint.setHeadingLocation(pt);
                    modifiedSinceSave = true;
                }
            }
            repaint();
        }

        @Override
        public void mouseMoved(MouseEvent e) {
            Point2D pt = mouse = (Point2D.Double) mouseXfm.transform(
                    new Point2D.Double(e.getPoint().getX(), e.getPoint().getY()), null);
            if (mode == MODE_EDIT) {
                testMouseOver(pt);
            }
            repaint();
        }

        /**
         * Determine whether the mouse is over anything that is significant for path editing. Significant
         * for path editing are:
         * <ul>
         *     <li>Control Points - control point tangent and heading handles, as well as control point
         *     field position</li>
         *     <li></li>
         * </ul>
         *
         * @param pt (Point2D, not null) The position of the mouse transformed to field coordinates.
         */
        private void testMouseOver(@NotNull Point2D pt) {
            overControlPoint = null;
            overPathPoint = null;
            overWhat = OVER_NOTHING;
            // loop through the control points and see is we are over a control point handle. These are tested
            // first as they are most important in controlling the path
            for (ControlPoint point : path.getControlPoints()) {
                // Normally we test the tangent point first, because it is rare that the tangents would be 0,
                // which would make the tangent point coincident with the control point and would stop the
                // robot at that point. The notable exception is when the robot stops to do something. In
                // that case the tangent is explicitly set to 0.0, and cannot be changed unless the stop and
                // do something action is removed.
                if ((null == point.getRobotAction()) &&
                        point.testOveTangentPoint(pt.getX(), pt.getY(), OVER_TOL / scale)) {
                    overControlPoint = point;
                    overWhat = OVER_TANGENT_POINT;

                } else if (point.testOverControlPoint(pt.getX(), pt.getY(), OVER_TOL / scale)) {
                    overControlPoint = point;
                    overWhat = OVER_CONTROL_POINT;

                } else if (point.testOverHeadingPoint(pt.getX(), pt.getY(), OVER_TOL / scale)) {
                    overControlPoint = point;
                    overWhat = OVER_HEADING_POINT;
                }
            }
            // If we are not over any  control point handle, then test path points to see if we are over
            // a path point. There are few actions for path points, however, sometimes we want something
            // to happen along the path.
            if (OVER_NOTHING == overWhat) {
                // Draw the path as a set of segments uniformly spaced in time.
                for (PathPoint pathPoint : path.getCurveSegments()) {
                    if (pathPoint.testOverPathPoint(pt.getX(), pt.getY(), OVER_TOL / scale)) {
                        overPathPoint = pathPoint;
                        overWhat = OVER_PATH_POINT;
                        break;
                    }
                }
            }
        }

        /**
         * Display the right mouse context menu. Appropriately enable/disable menu items based on the
         * current context.
         *
         * @param e The mouse event requesting this display - really used to determine where the context menu
         *          will be displayed.
         */
        private void displayContextMenu(MouseEvent e) {
            // check the current state of the path and editing state and enable choices that are valid in
            // the current state; disable choices that are invalid in the current state.
            menuItemStopAnimate.setEnabled(animate);

            menuSwitchAlliance.setEnabled(true);

            menuItemExtendPath.setEnabled(mode != MODE_ADD);
            menuItemEndPath.setEnabled(mode == MODE_ADD);

            boolean pathPointSelected = (null != overPathPoint) && (overWhat == OVER_PATH_POINT);
            menuItemInsert.setEnabled(pathPointSelected);

            boolean controlPointSelected = (null != overControlPoint) && (overWhat == OVER_CONTROL_POINT);
            menuItemDelete.setEnabled(controlPointSelected && (null != overControlPoint.getLast()));
            menuItemResetTangent.setEnabled(controlPointSelected && overControlPoint.getDerivativesManuallyEdited());
            menuItemSetTime.setEnabled(controlPointSelected && (null != overControlPoint.getLast()));
            menuItemInfo.setEnabled(controlPointSelected || pathPointSelected);

            contextMenu.show(e.getComponent(), e.getX(), e.getY());
        }
    }

    static private MenuItem pkgCreateMenuItem(PopupMenu menu, String name, ActionListener actionListener) {
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
        this.robot = robot;
        this.field = field;
        this.titleChange = titleChange;

        // build the right menu popup
        contextMenu = new PopupMenu();
        menuItemClearPath = pkgCreateMenuItem(contextMenu, "Clear Path", this);
        menuItemAnimatePath = pkgCreateMenuItem(contextMenu, "Play Path", this);
        menuItemStopAnimate = pkgCreateMenuItem(contextMenu, "Stop Play", this);
        contextMenu.addSeparator();
        menuSwitchAlliance = pkgCreateMenuItem(contextMenu, "Switch Alliance", this);
        contextMenu.addSeparator();
        menuItemExtendPath = pkgCreateMenuItem(contextMenu, "Extend Path", this);
        menuItemEndPath = pkgCreateMenuItem(contextMenu, "End Path", this);
        contextMenu.addSeparator();
        menuItemInsert = pkgCreateMenuItem(contextMenu, "Insert Control Point", this);
        menuItemDelete = pkgCreateMenuItem(contextMenu, "Delete Control Point", this);
        menuItemResetTangent = pkgCreateMenuItem(contextMenu, "Reset Tangent", this);
        menuItemSetTime = pkgCreateMenuItem(contextMenu, "Set Time", this);
        menuItemInfo = pkgCreateMenuItem(contextMenu, "Info", this);
        add(contextMenu);

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
        robotChassis = new GeneralPath(GeneralPath.WIND_NON_ZERO, 4);
        robotChassis.moveTo(-robot.getChassisWidth() / 2.0, -robot.getChassisLength() / 2.0);
        robotChassis.lineTo(-robot.getChassisWidth() / 2.0, robot.getChassisLength() / 2.0);
        robotChassis.lineTo(robot.getChassisWidth() / 2.0, robot.getChassisLength() / 2.0);
        robotChassis.lineTo(robot.getChassisWidth() / 2.0, -robot.getChassisLength() / 2.0);
        robotChassis.closePath();

        robotBumpers = new GeneralPath(GeneralPath.WIND_NON_ZERO, 4);
        robotBumpers.moveTo(-robot.getBumperWidth() / 2.0, -robot.getBumperLength() / 2.0);
        robotBumpers.lineTo(-robot.getBumperWidth() / 2.0, robot.getBumperLength() / 2.0);
        robotBumpers.lineTo(robot.getBumperWidth() / 2.0, robot.getBumperLength() / 2.0);
        robotBumpers.lineTo(robot.getBumperWidth() / 2.0, -robot.getBumperLength() / 2.0);
        robotBumpers.closePath();

        robotCorners[0].x = -robot.getBumperWidth() / 2.0;
        robotCorners[0].y = -robot.getBumperLength() / 2.0;
        robotCorners[1].x = -robot.getBumperWidth() / 2.0;
        robotCorners[1].y = robot.getBumperLength() / 2.0;
        robotCorners[2].x = robot.getBumperWidth() / 2.0;
        robotCorners[2].y = robot.getBumperLength() / 2.0;
        robotCorners[3].x = robot.getBumperWidth() / 2.0;
        robotCorners[3].y = -robot.getBumperLength() / 2.0;

    }

    /**
     * An action listener for the {@link PathCanvas} that is specifically looking for {@link #timer} events
     * during path animation.
     *
     * @param event (ActionEvent) The action event that was sent to this path canvas.
     */
    @Override
    public void actionPerformed(ActionEvent event) {
        final Object src = event.getSource();
        if ((event.getSource() == timer) && (null != pathFollower)) {
            // This is a timer event while there is a path follower.
            if (pathStartTime == -1) {
                 pathStartTime = event.getWhen();
            }
            if (stopAndRunEndTime < event.getWhen()) {
                if (null != stopAndRunDescription) {
                    stopAndRunEndTime = -1;
                    stopAndRunDescription = null;
                }
                // This is a point at some time on the path
                currentPathTime = (event.getWhen() - pathStartTime - stopAndRunDuration) / 1000.0;
                currentPathPoint = pathFollower.getPointAt(currentPathTime);
                if (null == currentPathPoint) {
                    pkgStopAnimation();
                    // reached the end of the path
                } else {
                    if ((null != currentPathPoint.action) &&
                            (RobotActionType.STOP_AND_RUN_COMMAND == currentPathPoint.action.actionType)) {
                        stopAndRunEndTime = event.getWhen() + (long) (currentPathPoint.action.approxDuration * 1000.0);
                        stopAndRunDescription = "Stop and Run: " + currentPathPoint.action.command;
                        stopAndRunDuration += (long) (currentPathPoint.action.approxDuration * 1000.0);
                        System.out.printf("    stopping to run: " + currentPathPoint.action.command + "%n");
                    }
                }
            }
            repaint();
        } else if (src == menuItemClearPath) {
            clearPath();
        } else if (src == menuItemAnimatePath) {
            animatePath();
        } else if (src == menuItemStopAnimate) {
            pkgStopAnimation();
            repaint();
        } else if (src == menuSwitchAlliance) {
            switchAlliance();
        } else if (src == menuItemEndPath) {
            pkgSetEditMode();
        } else if (src == menuItemExtendPath) {
            pkgSetExtendMode();
        } else if (src == menuItemInsert) {
            path.insertControlPoint(overPathPoint.time * path.getSpeedMultiplier());
            modifiedSinceSave = true;
            repaint();
        } else if (src == menuItemDelete) {
            path.deleteControlPoint(overControlPoint);
            overControlPoint = null;
            overWhat = OVER_NOTHING;
            modifiedSinceSave = true;
            repaint();
        } else if (src == menuItemResetTangent) {
            overControlPoint.resetDerivative();
            modifiedSinceSave = true;
            repaint();
        } else if (src == menuItemSetTime) {
            pkgControlPointTimeDialog();
            repaint();
        } else if (src == menuItemInfo) {
            if (overWhat == OVER_CONTROL_POINT) {
                pkgControlPointDialog();
            } else if (overWhat == OVER_PATH_POINT) {
                pkgPathPointDialog();
            }
            repaint();
        }
    }

    /**
     * This is a dialog that only displays and edits the time for  control point.
     */
    private void pkgControlPointTimeDialog() {
        JPanel p = new JPanel(new BorderLayout(5, 5));

        JPanel labels = new JPanel(new GridLayout(0, 1, 2, 2));
        JLabel labelTime = pkgLoadAndAddLabel(labels, "Time");
        p.add(labels, BorderLayout.LINE_START);

        JPanel controls = new JPanel(new GridLayout(0, 1, 2, 2));
        JTextField time = new JTextField(String.format("%.2f", overControlPoint.getTime()));
        controls.add(time);
        p.add(controls, BorderLayout.CENTER);

        int status = JOptionPane.showConfirmDialog(
                this, p, "Control Point Time:", JOptionPane.OK_CANCEL_OPTION);
        if (status == JOptionPane.OK_OPTION) {
            pkgSetTime(time, labelTime);
        }

    }

    /**
     * This is the dialog for control point info that lets you manually adjust position, derivatives,
     * time, and stop-and-run actions.
     */
    private void pkgControlPointDialog() {
        JPanel p = new JPanel(new BorderLayout(5, 5));

        JPanel labels = new JPanel(new GridLayout(0, 1, 2, 2));
        JLabel labelX = pkgLoadAndAddLabel(labels, "Field X");
        JLabel labelY = pkgLoadAndAddLabel(labels, "Field Y");
        JLabel label_dX = pkgLoadAndAddLabel(labels, "Field dX");
        JLabel label_dY = pkgLoadAndAddLabel(labels, "Field dY");
        JLabel labelHeading = pkgLoadAndAddLabel(labels, "Heading");
        JLabel labelTime = pkgLoadAndAddLabel(labels, "At Time");
        p.add(labels, BorderLayout.LINE_START);

        JPanel controls = new JPanel(new GridLayout(0, 1, 2, 2));
        JTextField fieldX = pkgLoadAndAddField(controls, overControlPoint.getFieldX(),"%.3f");
        JTextField fieldY = pkgLoadAndAddField(controls, overControlPoint.getFieldY(),"%.3f");
        JTextField field_dX = pkgLoadAndAddField(controls, overControlPoint.getRawTangentX(),"%.3f");
        JTextField field_dY = pkgLoadAndAddField(controls, overControlPoint.getRawTangentY(),"%.3f");
        JTextField heading = pkgLoadAndAddField(controls, overControlPoint.getFieldHeading(),"%.3f");
        JTextField time = pkgLoadAndAddField(controls, overControlPoint.getTime(),"%.2f");
        time.setEditable(null != overControlPoint.getLast());
        p.add(controls, BorderLayout.CENTER);

        JPanel stopAction = new JPanel(new BorderLayout(5, 5));
        JPanel stopOnOff = new JPanel(new GridLayout(0, 1, 2, 2));
        stopOnOff.add(new JSeparator(SwingConstants.HORIZONTAL));
        JCheckBox hasStopAction = new JCheckBox("stop and execute command");
        KochanekBartelsSpline.RobotAction robotAction = overControlPoint.getRobotAction();
        hasStopAction.setSelected(null != robotAction);
        stopOnOff.add(hasStopAction);
        stopAction.add(stopOnOff, BorderLayout.PAGE_START);
        JPanel stopLabels = new JPanel(new GridLayout(0, 1, 2, 2));
        JLabel labelStopCommand = pkgLoadAndAddLabel(stopLabels, "Command");
        JLabel labelStopDuration = pkgLoadAndAddLabel(stopLabels, "Duration");
        stopAction.add(stopLabels, BorderLayout.LINE_START);
        JPanel stopControls = new JPanel(new GridLayout(0, 1, 2, 2));
        JTextField fieldStopCommand = pkgLoadAndAddField(
                stopControls, (null == robotAction) ? "" : robotAction.command);
        JTextField fieldStopDuration = (null == robotAction) ?
                pkgLoadAndAddField(stopControls, ""):
                pkgLoadAndAddField(stopControls, robotAction.approxDuration,"%.3f");
        stopAction.add(stopControls, BorderLayout.CENTER);
        p.add(stopAction, BorderLayout.PAGE_END);

//        int status = JOptionPane.showConfirmDialog(
//                this, p, "Control Point Info:", JOptionPane.OK_CANCEL_OPTION);
        Object[] buttons = {"apply", "dismiss" };
        int status = JOptionPane.showOptionDialog(
                this, p, "Control Point Info:", JOptionPane.OK_CANCEL_OPTION,
                JOptionPane.PLAIN_MESSAGE,null, buttons, buttons[1]);
        if (status == JOptionPane.OK_OPTION) {
            // TODO - This part of the processing may only involve changes to position or time, and do not involve
            // TODO - setting the derivatives. There needs to be a check whether derivatives were changed/set
            // TODO - before calling the setTangent() function which locks the derivatives to the current value.
            overControlPoint.setFieldLocation(
                    pkgGetDoubleFromTextField(fieldX, labelX, overControlPoint.getFieldX()),
                    pkgGetDoubleFromTextField(fieldY, labelY, overControlPoint.getFieldY())
            );
            overControlPoint.setTangent(
                    pkgGetDoubleFromTextField(field_dX, label_dX, overControlPoint.getRawTangentX()),
                    pkgGetDoubleFromTextField(field_dY, label_dY, overControlPoint.getRawTangentY())
            );
            overControlPoint.setFieldHeading(
                    pkgGetAngleFromTextField(heading, labelHeading, overControlPoint.getFieldHeading())
            );
            pkgSetTime(time, labelTime);
            if (hasStopAction.isSelected()) {
                overControlPoint.setRobotAction(
                        fieldStopCommand.getText(),
                        pkgGetDoubleFromTextField(fieldStopDuration, labelStopDuration,0.0));
            } else {
                overControlPoint.setRobotAction(null,0.0);
            }
        }
    }

    private void pkgPathPointDialog() {
        JPanel p = new JPanel(new BorderLayout(5, 5));

        JPanel labels = new JPanel(new GridLayout(0, 1, 2, 2));
        JLabel labelX = pkgLoadAndAddLabel(labels, "Field X");
        JLabel labelY = pkgLoadAndAddLabel(labels, "Field Y");
        JLabel labelHeading = pkgLoadAndAddLabel(labels, "Heading");
        JLabel labelTime = pkgLoadAndAddLabel(labels, "At Time");
        p.add(labels, BorderLayout.LINE_START);

        JPanel controls = new JPanel(new GridLayout(0, 1, 2, 2));
        JLabel fieldX = pkgLoadAndAddLabel(controls, overPathPoint.fieldPt.getX(),"  %.3f");
        JLabel fieldY = pkgLoadAndAddLabel(controls, overPathPoint.fieldPt.getX(),"  %.3f");
        JLabel heading = pkgLoadAndAddLabel(controls, overPathPoint.fieldHeading.getRadians(),"  %.3f");
        JLabel time = pkgLoadAndAddLabel(controls, overPathPoint.time,"  %.2f");controls.add(time);
        p.add(controls, BorderLayout.CENTER);

        JPanel scheduleAction = new JPanel(new BorderLayout(5, 5));
        JPanel scheduleOnOff = new JPanel(new GridLayout(0, 1, 2, 2));
        scheduleOnOff.add(new JSeparator(SwingConstants.HORIZONTAL));
        JCheckBox hasScheduledAction = new JCheckBox("schedule command");
        KochanekBartelsSpline.RobotAction robotAction = overPathPoint.action;
        hasScheduledAction.setSelected(null != robotAction);
        scheduleOnOff.add(hasScheduledAction);
        scheduleAction.add(scheduleOnOff, BorderLayout.PAGE_START);
        JPanel stopLabels = new JPanel(new GridLayout(0, 1, 2, 2));
        JLabel labelScheduleCommand = pkgLoadAndAddLabel(stopLabels, "Command");
        scheduleAction.add(stopLabels, BorderLayout.LINE_START);
        JPanel stopControls = new JPanel(new GridLayout(0, 1, 2, 2));
        JTextField fieldScheduleCommand = pkgLoadAndAddField(
                stopControls, (null == robotAction) ? "" : robotAction.command);
        scheduleAction.add(stopControls, BorderLayout.CENTER);
        p.add(scheduleAction, BorderLayout.PAGE_END);

        Object[] buttons = {"apply", "dismiss" };
        int status = JOptionPane.showOptionDialog(
                this, p, "Path Point Info:", JOptionPane.OK_CANCEL_OPTION,
                JOptionPane.PLAIN_MESSAGE,null, buttons, buttons[1]);
        if (status == JOptionPane.OK_OPTION) {
             if (hasScheduledAction.isSelected()) {
                 if ((null != robotAction) && !fieldScheduleCommand.getText().equals(robotAction.command)) {
                     // the command name has changed - delete the old action
                     path.deleteScheduledCommand(robotAction);
                 }
                 if ((null == robotAction) || !fieldScheduleCommand.getText().equals(robotAction.command)) {
                     // there is no old action, of the command has changed and the old action was deleted,
                     // so schedule a new one.
                     path.scheduleCommand(overPathPoint.time, fieldScheduleCommand.getText());
                 }
            } else if (null != robotAction) {
                 path.deleteScheduledCommand(robotAction);
            }
        }
    }

    private void pkgSetTime(JTextField time, JLabel labelTime) {
        if (null != overControlPoint.getLast()) {
            overControlPoint.setTime(Utl.clip(
                    pkgGetDoubleFromTextField(time, labelTime, overControlPoint.getTime()),
                    overControlPoint.getLast().getTime() + 0.1,
                    (null == overControlPoint.getNext()) ?
                            Double.MAX_VALUE : (overControlPoint.getNext().getTime() - 0.1)),
                    true);
        }
    }

    private @NotNull JLabel pkgLoadAndAddLabel(@NotNull JPanel labels, String name) {
        JLabel label = new JLabel(name, SwingConstants.TRAILING);
        labels.add(label);
        return label;
    }

    private @NotNull JLabel pkgLoadAndAddLabel(JPanel labels, double value, String format) {
        String str = String.format(format, value);
        JLabel label = new JLabel(str, SwingConstants.LEADING);
        labels.add(label);
        return label;
    }

    private JTextField pkgLoadAndAddField(JPanel controls, double value, String format) {
        String str = String.format(format, value);
        JTextField field = new JTextField(str);
        controls.add(field);
        return field;
    }

    private JTextField pkgLoadAndAddField(JPanel controls, AngleConstantD value, String format) {
        String str = String.format(format, value.getRadians());
        JTextField field = new JTextField(str);
        controls.add(field);
        return field;
    }

    private JTextField pkgLoadAndAddField(JPanel controls, String value) {
        JTextField field = new JTextField(value);
        controls.add(field);
        return field;
    }

    private Double pkgGetDoubleFromTextField(JTextField field, JLabel label, double currentValue) {
        try {
            double newValue = Double.parseDouble(field.getText());
            if (newValue != currentValue) {
                modifiedSinceSave = true;
                currentValue = newValue;
            }
        } catch (NumberFormatException e) {
            JOptionPane.showMessageDialog(this,
                    String.format("In '%s': '%s' is not a valid number.", label, field.getText()));
        }
        return currentValue;
    }

    private AngleD pkgGetAngleFromTextField(JTextField field, JLabel label, AngleD currentAngle) {
        try {
            double newAngle = Double.parseDouble(field.getText());
            if (newAngle != currentAngle.getRadians()) {
                modifiedSinceSave = true;
                currentAngle.setRadians(newAngle);
            }
        } catch (NumberFormatException e) {
            JOptionPane.showMessageDialog(this,
                    String.format("In '%s': '%s' is not a valid angle.", label, field.getText()));
        }
        return currentAngle;
    }

    /**
     * This override does not do anything other than call {@link #paint(Graphics)}. The overridden method assumed
     * the update was drawing to the displayed video buffer, so it cleared the buffer and then drew the new content,
     * which results in a lot of screen flashing for older video cards.
     *
     * @param g The graphic context to be updated (repainted)
     */
    @Override
    public void update(Graphics g) {
        paint(g);
    }

    /**
     * Paint the panel, which in the double buffer context means:
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
            pkgResetBuffer();
        }
        if (bufferGraphics != null) {
            //this clears the back buffer
            bufferGraphics.clearRect(0, 0, bufferWidth, bufferHeight);

            // draw the content to the back buffer
            pkgPaintBuffer(bufferGraphics);

            // copy the back buffer into this displayed panel
            g.drawImage(bufferImage, 0, 0, this);
        }
    }

    /**
     * Create a back buffer that is the size of the panel.
     */
    private void pkgResetBuffer() {
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
    public void pkgPaintBuffer(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setPaint(Color.WHITE);

        // draw the field first, everything else is on top of the field
        field.draw(g2d, drawXfm);

        // draw the robot at the control points. otherwise, the robot obscures the path and
        // other control point editing handles.
        if (animate) {
            g2d.drawString(
                    String.format("elapsed time = %.3f", (System.currentTimeMillis() - pathStartTime)/1000.0),
                    10, 20);
            g2d.drawString(
                    String.format("path time = %.3f", currentPathTime), 10, 35);
            g2d.drawString(
                    String.format("forward = %.3f", currentPathPoint.speedForward), 10, 50);
            g2d.drawString(
                    String.format("strafe = %.3f", currentPathPoint.speedStrafe), 10, 65);
            g2d.drawString(
                    String.format("angular vel = %.3f", currentPathPoint.speedRotation), 10, 80);
            if (null != stopAndRunDescription) {
                g2d.drawString(stopAndRunDescription, 10, 95);
            }
            boolean tooFast = !robot.canRobotAchieve(currentPathPoint.speedForward,
                    currentPathPoint.speedStrafe, currentPathPoint.speedRotation);
            System.out.printf("%10.3f, %10.3f, %10.3f, %10.3f      %b %n",
                    currentPathTime, currentPathPoint.speedForward,
                    currentPathPoint.speedStrafe, currentPathPoint.speedRotation, tooFast);
            pkgPaintRobot(g2d, currentPathPoint, tooFast);
            g2d.setPaint(Color.MAGENTA);
            double fieldX = currentPathPoint.fieldPt.getX();
            double fieldY = currentPathPoint.fieldPt.getY();
            Point2D.Double fieldPt = (Point2D.Double) drawXfm.transform(
                    new Point2D.Double(fieldX, fieldY), null);
            double dirX = fieldX + currentPathPoint.fieldHeading.sin();
            double dirY = fieldY + currentPathPoint.fieldHeading.cos();
            Point2D.Double dirPt = (Point2D.Double) drawXfm.transform(
                    new Point2D.Double(dirX, dirY), null);
            g2d.drawLine((int) fieldPt.getX(), (int) fieldPt.getY(), (int) dirPt.getX(), (int) dirPt.getY());
            g2d.drawOval((int) dirPt.getX() - 3, (int) dirPt.getY() - 3, 6, 6);

        } else {
            for (ControlPoint point : path.getControlPoints()) {
                pkgPaintRobot(g2d, point);
            }
        }
        g2d.setPaint(Color.WHITE);

        // Draw the path as a set of segments uniformly spaced in time.
        PathPoint lastPathPoint;
        Point2D.Double lastPt;
        PathPoint thisPathPoint = null;
        Point2D.Double thisPt = null;
        for (PathPoint pathPoint : path.getCurveSegments()) {
            lastPathPoint = thisPathPoint;
            lastPt = thisPt;
            thisPathPoint = pathPoint;
            boolean tooFast = !robot.canRobotAchieve(pathPoint.speedForward,
                    pathPoint.speedStrafe, pathPoint.speedRotation);

            g2d.setPaint(pkgIsRobotInside(pathPoint.fieldPt, pathPoint.fieldHeading) ?
                    (tooFast ? Color.RED : Color.WHITE) : Color.ORANGE);
            thisPt = (Point2D.Double) drawXfm.transform(pathPoint.fieldPt, null);

            if (lastPathPoint != null) {
                g2d.drawLine((int) lastPt.getX(), (int) lastPt.getY(),
                        (int) thisPt.getX(), (int) thisPt.getY());
            }
            if (null == pathPoint.action) {
                g2d.drawOval((int) thisPt.getX() - 2, (int) thisPt.getY() - 2, 4, 4);
            } else if (RobotActionType.SCHEDULE_COMMAND == pathPoint.action.actionType) {
                pkgDrawScheduledRobotAction(g2d, thisPt, true);
            } else {
                pkgDrawStopAndRunRobotAction(g2d, thisPt, true);
            }
        }

        if (!animate) {
            // draw the control point editing handles.
            for (ControlPoint point : path.getControlPoints()) {
                g2d.setPaint(Color.RED);
                KochanekBartelsSpline.RobotAction robotAction = point.getRobotAction();
                Point2D.Double fieldPt = (Point2D.Double) drawXfm.transform(
                        new Point2D.Double(point.getFieldX(), point.getFieldY()), null);

                // draw the control point
                if (null == robotAction) {
                    g2d.drawOval((int) fieldPt.getX() - 3, (int) fieldPt.getY() - 3, 6, 6);
                    g2d.fillOval((int) fieldPt.getX() - 3, (int) fieldPt.getY() - 3, 6, 6);
                } else {
                    pkgDrawStopAndRunRobotAction(g2d, fieldPt, true);
                }
                // draw the tangent handle
                if (null == robotAction) {
                    Point2D.Double tangentPt = (Point2D.Double) drawXfm.transform(
                            new Point2D.Double(point.getTangentX(), point.getTangentY()), null);
                    g2d.drawLine((int) fieldPt.getX(), (int) fieldPt.getY(),
                            (int) tangentPt.getX(), (int) tangentPt.getY());
                    g2d.drawOval((int) tangentPt.getX() - 3, (int) tangentPt.getY() - 3, 6, 6);
                }
                // draw the heading handle
                Point2D.Double headingPt = (Point2D.Double) drawXfm.transform(
                        new Point2D.Double(point.getHeadingX(), point.getHeadingY()), null);
                g2d.setPaint(Color.MAGENTA);
                g2d.drawLine((int) fieldPt.getX(), (int) fieldPt.getY(), (int) headingPt.getX(), (int) headingPt.getY());
                g2d.drawOval((int) headingPt.getX() - 3, (int) headingPt.getY() - 3, 6, 6);
            }

            // If the cursor is over a control point or path point, highlight it
            if (overWhat != OVER_NOTHING) {
                Stroke oldStroke = g2d.getStroke();
                g2d.setStroke(highlightStroke);
                g2d.setPaint(overWhat == OVER_PATH_POINT ? Color.ORANGE : Color.GREEN);
                switch (overWhat) {
                    case OVER_CONTROL_POINT:
                        pkgDrawFieldPointHighlight(g2d,
                                overControlPoint.getFieldX(), overControlPoint.getFieldY(),
                                overControlPoint.getRobotAction());
                        break;
                    case OVER_TANGENT_POINT:
                        pkgDrawFieldPointHighlight(g2d,
                                overControlPoint.getTangentX(), overControlPoint.getTangentY(), null);
                        break;
                    case OVER_HEADING_POINT:
                        pkgDrawFieldPointHighlight(g2d,
                                overControlPoint.getHeadingX(), overControlPoint.getHeadingY(), null);
                        break;
                    case OVER_PATH_POINT:
                        pkgDrawFieldPointHighlight(g2d,
                                overPathPoint.fieldPt.getX(), overPathPoint.fieldPt.getY(),
                                overPathPoint.action);
                        break;

                }
                g2d.setStroke(oldStroke);
            }

            // draw the mouse and tracking info
            // TODO: handle repositioning the text when the cursor gets to the edge of
            //  the window.
            if (null != mouse) {
                g2d.setPaint(Color.WHITE);
                Point2D screenMouse = drawXfm.transform(mouse, null);
                g2d.drawString(
                        String.format(" (%.4f,%.4f)", mouse.getX(), mouse.getY()),
                        (int) screenMouse.getX(), (int) screenMouse.getY());
            }
        }
    }

    private void pkgDrawFieldPointHighlight(Graphics2D g2d, double fieldX, double fieldY,
                                            KochanekBartelsSpline.RobotAction robotAction) {
        Point2D.Double fieldPt = (Point2D.Double) drawXfm.transform(
                new Point2D.Double(fieldX, fieldY), null);
        if (null == robotAction) {
            g2d.drawOval((int) fieldPt.getX() - 4, (int) fieldPt.getY() - 4, 8, 8);
        } else if (RobotActionType.STOP_AND_RUN_COMMAND == robotAction.actionType) {
            pkgDrawStopAndRunRobotAction(g2d, fieldPt, false);
        } else if (RobotActionType.SCHEDULE_COMMAND == robotAction.actionType) {
            pkgDrawScheduledRobotAction(g2d, fieldPt, false);
        }
    }

    private void pkgDrawStopAndRunRobotAction(Graphics2D g2d, Point2D.Double fieldPt, boolean fill) {
        int[] tmpX = new int[6];
        int[] tmpY = new int[6];
        for (int i = 0; i < 6; i++) {
            tmpX[i] = robotStopAndRunActionX[i] + (int)fieldPt.getX();
            tmpY[i] = robotStopAndRunActionY[i] + (int)fieldPt.getY();
        }
        g2d.drawPolygon(tmpX, tmpY, 6);
        if (fill) {
            g2d.fillPolygon(tmpX, tmpY, 6);
        }
    }

    private void pkgDrawScheduledRobotAction(Graphics2D g2d, Point2D.Double fieldPt, boolean fill) {
        int[] tmpX = new int[4];
        int[] tmpY = new int[4];
        for (int i = 0; i < 4; i++) {
            tmpX[i] = robotScheduleActionX[i] + (int)fieldPt.getX();
            tmpY[i] = robotScheduleActionY[i] + (int)fieldPt.getY();
        }
        g2d.drawPolygon(tmpX, tmpY, 4);
        if (fill) {
            g2d.fillPolygon(tmpX, tmpY, 4);
        }
    }

    private void pkgPaintRobot(Graphics2D g2d, ControlPoint controlPoint) {
        pkgPaintRobot(g2d, new Point2D.Double(controlPoint.getFieldX(), controlPoint.getFieldY()),
                controlPoint.getFieldHeading(), false);

    }

    private void pkgPaintRobot(Graphics2D g2d, PathPoint pathPoint, boolean tooFast) {
        pkgPaintRobot(g2d, pathPoint.fieldPt, pathPoint.fieldHeading, tooFast);


    }

    private void pkgPaintRobot(Graphics2D g2d, Point2D fieldPt, AngleConstantD heading, boolean tooFast) {
        AffineTransform oldXfm = g2d.getTransform();
        AffineTransform xfm = new AffineTransform(oldXfm);
        xfm.concatenate(drawXfm);

        AffineTransform xfmRobot = new AffineTransform();
        xfmRobot.translate(fieldPt.getX(), fieldPt.getY());
        xfmRobot.rotate(-heading.getRadians());
        xfm.concatenate(xfmRobot);
        // don't know why this scale is required, it should be on the oldXfm or the field rendering
        //  would be wrong ---- TODO. figure this out.
        xfm.scale(0.5, 0.5);

        xfmRobot.transform(robotCorners, 0, xfmRobotCorners, 0, 4);
        boolean inside = field.isInsideField(xfmRobotCorners, 0.05);

        g2d.setTransform(xfm);
        g2d.setPaint(inside ? (tooFast ? Color.RED : Color.MAGENTA) : Color.ORANGE);
        g2d.draw(robotBumpers);
        g2d.fill(robotBumpers);
        g2d.setPaint(Color.BLACK);
        g2d.draw(robotChassis);
        g2d.fill(robotChassis);
        g2d.setTransform(oldXfm);
    }

    private boolean pkgIsRobotInside(Point2D fieldPt, AngleConstantD heading) {
        AffineTransform xfmRobot = new AffineTransform();
        xfmRobot.translate(fieldPt.getX(), fieldPt.getY());
        xfmRobot.rotate(-heading.getRadians());
        xfmRobot.transform(robotCorners, 0, xfmRobotCorners, 0, 4);
        return field.isInsideField(xfmRobotCorners, 0.05);
    }

    private void pkgSetEditMode() {
        newControlPoint = null;
        overControlPoint = null;
        mode = MODE_EDIT;
        overWhat = OVER_NOTHING;
    }

    private void pkgSetExtendMode() {
        newControlPoint = null;
        overControlPoint = null;
        mode = MODE_ADD;
        overWhat = OVER_NOTHING;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Public functions called by PathPlanner main menu
    // -----------------------------------------------------------------------------------------------------------------

    /**
     * Get the path being edited.
     *
     * @return The path.
     */
    @NotNull
    public KochanekBartelsSpline getPath() {
        return path;
    }

    /**
     * Get the name of the path filename.
     *
     * @return {@code null} if a filename has not been set, otherwise the name of the current
     * path file.
     */
    @Nullable
    public File getPathFile() {
        return pathFile;
    }

    /**
     * Test whether the path been modified since the last save.
     *
     * @return {@code true} the last changes have not been saved, {@code false} if there have
     * been no path changes since the last path save.
     */
    public boolean modifiedSinceSave() {
        return modifiedSinceSave;
    }

    /**
     * Start a new path. This means clear the current path (all the control points, etc.) and
     * restart creating a path. This is different from clearing a path which maintains the context
     * of the path.
     */
    public void newPath() {
        clearPath();
        pathFile = null;
        modifiedSinceSave = false;
        titleChange.titleChanged();
    }

    /**
     * Clear the current path and restart drawing that path.
     */
    public void clearPath() {
        path.clearPath();
        pkgSetExtendMode();
        pathFile = null;
        modifiedSinceSave = false;
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
            pathFile = file;
            path.loadPath(file.getAbsolutePath());
            modifiedSinceSave = false;
            titleChange.titleChanged();
        } else {
            System.out.println("Load path command cancelled by user.");
        }
        pkgSetEditMode();
        repaint();
    }

    /**
     * The menu action to reload a path. Very useful if you are hand editing a path.
     */
    public void reloadPath() {
        System.out.println("Reloading path from: " + pathFile.getAbsolutePath());
        path.loadPath(pathFile.getAbsolutePath());
        modifiedSinceSave = false;
        pkgSetEditMode();
        repaint();
    }

    /**
     * Save the path file to the current path file.
     */
    public void savePath() {
        System.out.println("Saving path as: " + pathFile.getAbsolutePath());
        path.savePath(pathFile.getAbsolutePath());
        modifiedSinceSave = false;
        pkgSetEditMode();
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
            pathFile = fc.getSelectedFile();
            if (!pathFile.getAbsolutePath().endsWith(".json")) {
                pathFile = new File(pathFile.getAbsolutePath() + ".json");
            }
            savePath();
            titleChange.titleChanged();
        } else {
            System.out.println("Save path command cancelled by user.");
        }
    }

    /**
     * Animate the robot position on the path from start to end of the path.
     */
    public void animatePath() {
        if (null == timer) {
            // create the timer if one does not exist
            timer = new Timer(20, this);
            timer.setInitialDelay(200);
            timer.start();
        } else {
            timer.restart();
        }
        animate = true;
        pathStartTime = -1;
        currentPathTime = 0.0;
        stopAndRunEndTime = -1;
        stopAndRunDescription = null;
        stopAndRunDuration = 0;
        pathFollower = path.getPathFollower();
        System.out.printf("    seconds     forward      strafe     angular    too fast!%n");
    }

    /**
     * Stop an animation, normally called when the animation reaches the end of the path.
     */
    private void pkgStopAnimation() {
        timer.stop();
        animate = false;
        pathStartTime = -1;
        currentPathTime = 0.0;
        stopAndRunEndTime = -1;
        stopAndRunDescription = null;
        stopAndRunDuration = 0;
        pathFollower = null;
        currentPathPoint = null;
    }

    /**
     * Flip the alliance of the path. Practically this means go through all the control points adn flip ths
     * sign of X and X components.
     */
     void switchAlliance() {
        // So, the deal here is that we have built a path for the opposing alliance rather than
        // ours, and we want to flip it. we do that by going through the control points and flipping
        // the sign of anything X and Y (which is a 180deg rotation).
        for (ControlPoint cp : path.getControlPoints()) {
            if (cp.getDerivativesManuallyEdited()) {
                cp.setTangent(-cp.getRawTangentX(),-cp.getRawTangentY());
            }
            cp.setFieldLocation(-cp.getFieldX(), -cp.getFieldY());
            cp.setFieldHeading(cp.getFieldHeading().add(AngleD.PI));
        }
        repaint();
    }
}
