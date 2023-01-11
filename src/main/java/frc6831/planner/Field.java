package frc6831.planner;

import org.a05annex.util.geo2d.Plane2d;
import org.jetbrains.annotations.NotNull;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.ParseException;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.GeneralPath;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import static org.a05annex.util.JsonSupport.*;

/**
 * This class maintains and draws the field we will be plotting the path on. Note that the default
 * with no loaded field data is to draw the axes and a dotted outline of the field.
 */
@SuppressWarnings("HungarianNotationMemberVariables")
public class Field {

    // -------------------------------------------------------------------------------------------
    // Parsing the field file
    // -------------------------------------------------------------------------------------------
    // primary keys (sections) in the field description
    private static final String TITLE = "title";
    private static final String DESCRIPTION = "description";
    private static final String ARENA = "arena";
    private static final String COMPONENTS = "components";
    private static final String FIELD = "field";

    // elements in the arena
    private static final String EXTENT = "extent";
    private static final String VIEW = "view";

    // elements in th description of components for the game
    private static final String NAME = "name";
    private static final String LINE_COLOR = "lineColor";
    private static final String FILL_COLOR = "fillColor";
    private static final String SHAPES = "shapes";

    // The elements in the definition of a shapes
    private static final String TYPE = "type";

    private static final String TYPE_CIRCLE = "circle";
    private static final String CIRCLE_CENTER = "center";
    private static final String CIRCLE_RADIUS = "radius";

    private static final String TYPE_RECT = "rect";
    private static final String RECT_LOWER_LEFT = "lower left";
    private static final String RECT_UPPER_RIGHT = "upper right";

    private static final String TYPE_POLYGON = "polygon";
    private static final String POINTS = "points";

    private static final String COMPONENT = "component";
    private static final String ALLIANCE = "alliance";
    private static final String TRANSLATE = "translate";
    private static final String ROTATE = "rotate";
    private static final String SCALE = "scale";

    // -------------------------------------------------------------------------------------------
    // The field outline (extent) and view. Note that this is set to the 2022 Rapid React field
    // and view by default unless specified in the field description file.
    private final double AXIS_MARGIN = 0.4;
    // The 2022 field boundary - this may be overwritten by the field description
    private double X_FIELD_MIN = -4.115;
    private double Y_FIELD_MIN = -8.23;
    private double X_FIELD_MAX = 4.115;
    private double Y_FIELD_MAX = 8.23;
    // The 2022 field boundary - this may be overwritten by the field description
    private double X_VIEW_MIN = -4.115;
    private double Y_VIEW_MIN = -8.23;
    private double X_VIEW_MAX = 4.115;
    private double Y_VIEW_MAX = 8.23;

    private final String m_default_title = "2022 field";
    private final String m_default_description = "The 2022 field outline with no game elements.";

    // Additional colors - for 2021 Infinite Recharge at home
    private static final Color GREEN_ZONE = new Color(118, 215, 196);
    private static final Color YELLOW_ZONE = new Color(247, 220, 111);
    private static final Color BLUE_ZONE = new Color(133, 193, 233);
    private static final Color PURPLE_ZONE = new Color(187, 143, 206);
    private static final Color RED_ZONE = new Color(236, 112, 99);

    // the axis lines - probably resized whenever a field description is read.
    private final Point2D.Double X_AXIS_START = new Point2D.Double(X_VIEW_MIN - AXIS_MARGIN, 0.0);
    private final Point2D.Double X_AXIS_END = new Point2D.Double(X_VIEW_MAX + AXIS_MARGIN, 0.0);
    private final Point2D.Double Y_AXIS_START = new Point2D.Double(0.0, Y_VIEW_MIN - AXIS_MARGIN);
    private final Point2D.Double Y_AXIS_END = new Point2D.Double(0.0, Y_VIEW_MAX + AXIS_MARGIN);

    // the un-transformed field outline.
    private final Point2D.Double[] FIELD_OUTLINE = {
            new Point2D.Double(X_FIELD_MIN, Y_FIELD_MIN),
            new Point2D.Double(X_FIELD_MIN, Y_FIELD_MAX),
            new Point2D.Double(X_FIELD_MAX, Y_FIELD_MAX),
            new Point2D.Double(X_FIELD_MAX, Y_FIELD_MIN)
    };

    // the transformed (to screen coordinate) field outline.
    private final Point2D.Double[] m_xfmField = {
            new Point2D.Double(X_FIELD_MIN, Y_FIELD_MIN),
            new Point2D.Double(X_FIELD_MIN, Y_FIELD_MAX),
            new Point2D.Double(X_FIELD_MAX, Y_FIELD_MAX),
            new Point2D.Double(X_FIELD_MAX, Y_FIELD_MIN)
    };

    private final Plane2d[] fieldPlanes = {
            new Plane2d(1.0, 0.0, -X_FIELD_MAX),
            new Plane2d(-1.0, 0.0, X_FIELD_MIN),
            new Plane2d(0.0, 1.0, -Y_FIELD_MAX),
            new Plane2d(0.0, -1.0, Y_FIELD_MIN)
    };

    private final MinMax m_minMax =
            new MinMax(X_VIEW_MIN - AXIS_MARGIN, Y_VIEW_MIN - AXIS_MARGIN,
                    X_VIEW_MAX + AXIS_MARGIN, Y_VIEW_MAX + AXIS_MARGIN);

    private String m_title = m_default_title;
    private String m_description = m_default_description;
    private final HashMap<String, FieldComponent> m_components = new HashMap<>();
    private final ArrayList<FieldDraw> m_drawList = new ArrayList<>();


    // -------------------------------------------------------------------------------------------

    /**
     *
     */
    public static class MinMax {
        private double m_minX;
        private double m_minY;
        private double m_maxX;
        private double m_maxY;

        public MinMax(double minX, double minY, double maxX, double maxY) {
            setValue(minX, minY, maxX, maxY);
        }

        public void setValue(double minX, double minY, double maxX, double maxY) {
            m_minX = minX;
            m_minY = minY;
            m_maxX = maxX;
            m_maxY = maxY;
        }

        double getMinX() {
            return m_minX;
        }
        double getMinY() {
            return m_minY;
        }
        double getMaxX() {
            return m_maxX;
        }
        double getMaxY() {
            return m_maxY;
        }
    }

    // -------------------------------------------------------------------------------------------
    // Shapes that can be drawn to the field
    // -------------------------------------------------------------------------------------------
    public static abstract class FieldShape {
        FieldShape m_next = null;

        void setNext(FieldShape nextShape) {
            m_next = nextShape;
        }

        FieldShape getNext() {
            return m_next;
        }

        abstract void draw(Graphics2D g2d, AffineTransform drawXfm, Color outline, Color fill);
    }

    private static class FieldCircle extends FieldShape {

        Point2D m_center = new Point2D.Double(0.0, 0.0);
        double m_radius = 1.0;


        FieldCircle(JSONObject shapeDesc) {
            m_center = parsePoint(shapeDesc, CIRCLE_CENTER);
            m_radius = parseDouble(shapeDesc, CIRCLE_RADIUS, 1.0);
        }

        @Override
        void draw(Graphics2D g2d, AffineTransform drawXfm, Color outline, Color fill) {
            Point2D.Double ptCenter = (Point2D.Double) drawXfm.transform(m_center, null);
            double scale = Math.sqrt((drawXfm.getScaleX() * drawXfm.getScaleX()) +
                    (drawXfm.getShearX() * drawXfm.getShearX()));
            double scaledRadius = scale * m_radius;
            if (null != outline) {
                g2d.setPaint(outline);
                g2d.drawOval((int) (ptCenter.getX() - scaledRadius), (int) (ptCenter.getY() - scaledRadius),
                        (int) (2.0 * scaledRadius), (int) (2.0 * scaledRadius));
            }
            if (null != fill) {
                g2d.setPaint(fill);
                g2d.fillOval((int) (ptCenter.getX() - scaledRadius), (int) (ptCenter.getY() - scaledRadius),
                        (int) (2.0 * scaledRadius), (int) (2.0 * scaledRadius));
            }

        }
    }

    private static class FieldRect extends FieldShape {
        Point2D m_LL;
        final Point2D m_UR;

        FieldRect(JSONObject shapeDesc) {
            m_LL = parsePoint(shapeDesc, RECT_LOWER_LEFT);
            m_UR = parsePoint(shapeDesc, RECT_UPPER_RIGHT);

        }

        @Override
        void draw(Graphics2D g2d, AffineTransform drawXfm, Color outline, Color fill) {
            Point2D.Double ptLL = (Point2D.Double) drawXfm.transform(m_LL, null);
            Point2D.Double ptUR = (Point2D.Double) drawXfm.transform(m_UR, null);
            int x, y, width, height;
            if (ptLL.x < ptUR.x) {
                x = (int) ptLL.x;
                width = (int) (ptUR.x - ptLL.x);
            } else {
                x = (int) ptUR.x;
                width = (int) (ptLL.x - ptUR.x);

            }
            if (ptLL.y < ptUR.y) {
                y = (int) ptLL.y;
                height = (int) (ptUR.y - ptLL.y);
            } else {
                y = (int) ptUR.y;
                height = (int) (ptLL.y - ptUR.y);

            }
            if (null != outline) {
                g2d.setPaint(outline);
                g2d.drawRect(x, y, width, height);
            }
            if (null != fill) {
                g2d.setPaint(fill);
                g2d.fillRect(x, y, width, height);
            }
        }
    }

    private static class FieldPolygon extends FieldShape {
        final Point2D[] m_pts;
        final Point2D[] m_xfmPts;

        FieldPolygon(JSONObject shapeDesc) {
            JSONArray ptList = getJSONArray(shapeDesc, POINTS);
            int index = 0;
            m_pts = new Point2D[ptList.size()];
            m_xfmPts = new Point2D[ptList.size()];
            for (Object ptObj : ptList) {
                m_pts[index++] = parsePoint((JSONArray) ptObj);
            }
        }

        @Override
        void draw(Graphics2D g2d, AffineTransform drawXfm, Color outline, Color fill) {
            drawXfm.transform(m_pts, 0, m_xfmPts, 0, m_pts.length);
            GeneralPath polyPath = new GeneralPath(GeneralPath.WIND_NON_ZERO, 4);
            for (int i = 0; i < m_xfmPts.length; i++) {
                if (i == 0) {
                    polyPath.moveTo(m_xfmPts[i].getX(), m_xfmPts[i].getY());
                } else {
                    polyPath.lineTo(m_xfmPts[i].getX(), m_xfmPts[i].getY());
                }
            }
            polyPath.closePath();
            if (null != outline) {
                g2d.setPaint(outline);
                g2d.draw(polyPath);
            }
            if (null != fill) {
                g2d.setPaint(fill);
                g2d.fill(polyPath);
            }

        }
    }

    // -------------------------------------------------------------------------------------------
    // Components that may appear on the field multiple times, and optionally in alliance colors
    // -------------------------------------------------------------------------------------------
    private static class FieldComponent {
        String m_name = "default";
        String m_outlineColor = "white";
        String m_fillColor = null;
        FieldShape m_shape = null;

        public FieldComponent(JSONObject componentDesc) {
            m_name = parseString(componentDesc, NAME, m_name);
            m_outlineColor = parseString(componentDesc, LINE_COLOR, m_outlineColor);
            m_fillColor = parseString(componentDesc, FILL_COLOR, m_fillColor);

            JSONArray shapeList = getJSONArray(componentDesc, SHAPES);
            FieldShape lastShape = null;
            for (Object shape : shapeList) {
                FieldShape fieldShape = shapeFactory((JSONObject) shape);
                if (null != fieldShape) {
                    if (null == lastShape) {
                        m_shape = fieldShape;
                    } else {
                        lastShape.setNext(fieldShape);
                    }
                    lastShape = fieldShape;
                }
            }
        }

        public String getName() {
            return m_name;
        }

        public void draw(Graphics2D g2d, AffineTransform drawXfm, Color allianceColor) {
            FieldShape nextShape = m_shape;
            while (null != nextShape) {
                nextShape.draw(g2d, drawXfm,
                        (null == m_outlineColor) ? null : getColor(m_outlineColor, null, allianceColor),
                        (null == m_fillColor) ? null : getColor(m_fillColor, null, allianceColor));
                nextShape = nextShape.getNext();
            }
        }
    }

    // ----------------------------------------------------------------------------------------------------
    // The description
    // ----------------------------------------------------------------------------------------------------

    /**
     * A field element is a component that is positioned and drawn onto the field. This component is
     * optionally alliance color coded.
     */
    static class FieldDraw {
        final FieldComponent m_component;
        final AffineTransform m_xfm;
        final Color m_allianceColor;

        public FieldDraw(FieldComponent component, AffineTransform xfm, Color allianceColor) {
            m_component = component;
            m_xfm = xfm;
            m_allianceColor = allianceColor;
        }

        public void draw(Graphics2D g2d, AffineTransform drawXfm) {
            AffineTransform xfm = new AffineTransform(drawXfm);
            xfm.concatenate(m_xfm);
            m_component.draw(g2d, xfm, m_allianceColor);
        }

    }

    // ====================================================================================================
    // ====================================================================================================
    // Field - the actual implementation of the field class
    // ====================================================================================================
    // ====================================================================================================
    public String getTitle() {
        return m_title;
    }

    @SuppressWarnings("unused")
    public String getDescription() {
        return m_description;
    }

    /**
     * Get the field MinMax, which is used primarily during app resizing to make sure the whole
     * screen is in the window.
     *
     * @return The field min-max
     */
    public @NotNull MinMax getMinMax() {
        return m_minMax;
    }

    // ----------------------------------------------------------------------------------------------------
    // Loading from a JSON file
    // ----------------------------------------------------------------------------------------------------
    private void setDefaultEmptyField()
    {
        m_title = m_default_title;
        m_description = m_default_description;
        m_components.clear();
        m_drawList.clear();

    }

    private void resetExtentAndViewDependencies()
    {
        // reset the axis endpoints
        X_AXIS_START.setLocation(X_VIEW_MIN - AXIS_MARGIN, 0.0);
        X_AXIS_END.setLocation(X_VIEW_MAX + AXIS_MARGIN, 0.0);
        Y_AXIS_START.setLocation(0.0, Y_VIEW_MIN - AXIS_MARGIN);
        Y_AXIS_END.setLocation(0.0, Y_VIEW_MAX + AXIS_MARGIN);
        // field outline
        FIELD_OUTLINE[0].setLocation(X_FIELD_MIN, Y_FIELD_MIN);
        FIELD_OUTLINE[1].setLocation(X_FIELD_MIN, Y_FIELD_MAX);
        FIELD_OUTLINE[2].setLocation(X_FIELD_MAX, Y_FIELD_MAX);
        FIELD_OUTLINE[3].setLocation(X_FIELD_MAX, Y_FIELD_MIN);
        // field transformation
        m_xfmField[0].setLocation(X_FIELD_MIN, Y_FIELD_MIN);
        m_xfmField[1].setLocation(X_FIELD_MIN, Y_FIELD_MAX);
        m_xfmField[2].setLocation(X_FIELD_MAX, Y_FIELD_MAX);
        m_xfmField[3].setLocation(X_FIELD_MAX, Y_FIELD_MIN);
        // the transformed (to screen coordinate) field outline.
        fieldPlanes[0].setValue(1.0, 0.0, -X_FIELD_MAX);
        fieldPlanes[1].setValue(-1.0, 0.0, X_FIELD_MIN);
        fieldPlanes[2].setValue(0.0, 1.0, -Y_FIELD_MAX);
        fieldPlanes[3].setValue(0.0, -1.0, Y_FIELD_MIN);
        // The field min-max used in resizing.
        m_minMax .setValue(X_VIEW_MIN - AXIS_MARGIN, Y_VIEW_MIN - AXIS_MARGIN,
                        X_VIEW_MAX + AXIS_MARGIN, Y_VIEW_MAX + AXIS_MARGIN);
    }

    /** Load a field.
     *
     * @param filepath The path to the file ccontaining the field description.
     */
    public void loadField(@NotNull String filepath) {
        setDefaultEmptyField();
        try {
            JSONObject dict = readJsonFileAsJSONObject(filepath);
            // title and description
            m_title = parseString(dict, TITLE, "untitled");
            m_description = parseString(dict, DESCRIPTION, "No description provided.");
            // Arena - the basic field extent and view.
            JSONObject arena = getJSONObject(dict, ARENA,false);
            if (null != arena) {
                JSONArray extentList = getJSONArray(arena, EXTENT);
                if (4 == extentList.size()) {
                    X_FIELD_MIN = X_VIEW_MIN = (double)extentList.get(0);
                    Y_FIELD_MIN = Y_VIEW_MIN = (double)extentList.get(1);
                    X_FIELD_MAX = X_VIEW_MAX = (double)extentList.get(2);
                    Y_FIELD_MAX = Y_VIEW_MAX = (double)extentList.get(3);
                }
                JSONArray viewList = getJSONArray(arena, VIEW, false);
                if ((null != viewList) && (4 == viewList.size())) {
                    X_VIEW_MIN = (double)viewList.get(0);
                    Y_VIEW_MIN = (double)viewList.get(1);
                    X_VIEW_MAX = (double)viewList.get(2);
                    Y_VIEW_MAX = (double)viewList.get(3);
                }
            }
            resetExtentAndViewDependencies();
            // Read the field components
            JSONArray componentList = getJSONArray(dict, COMPONENTS, false);
            if (null != componentList) {
                for (Object component : componentList) {
                    if (component.getClass() == JSONObject.class) {
                        FieldComponent thisComponent = new FieldComponent((JSONObject) component);
                        m_components.put(thisComponent.getName(), thisComponent);
                    }
                }
            }
            // Read the field description
            JSONObject fieldDesc = getJSONObject(dict, FIELD);
            JSONArray drawList = getJSONArray(fieldDesc, COMPONENTS, false);
            if (null != drawList) {
                // read the components that should be drawn to represent the field.
                for (Object drawObj : drawList) {
                    if (drawObj.getClass() == JSONObject.class) {
                        JSONObject drawDesc = (JSONObject) drawObj;
                        // get the component
                        String componentName = parseString(drawDesc, COMPONENT, null);
                        FieldComponent component = m_components.get(componentName);
                        if (null == component) {
                            throw new NullPointerException(String.format("No component named '%s'", componentName));
                        }
                        // build the positioning transform
                        AffineTransform xfm = new AffineTransform();
                        double rotate = parseDouble(drawDesc, ROTATE, 0.0);
                        if (0.0 != rotate) {
                            xfm.rotate(rotate);
                        }
                        Point2D translate = parsePoint(drawDesc, TRANSLATE);
                        if (null != translate) {
                            xfm.translate(translate.getX(), translate.getY());
                        }
                        Point2D scale = parsePoint(drawDesc, SCALE);
                        if (null != scale) {
                            xfm.scale(scale.getX(), scale.getY());
                        }
                        // and set the alliance color (if there is one)
                        String colorName = parseString(drawDesc, ALLIANCE, null);
                        Color allianceColor = (null == colorName) ? null : getColor(colorName, null, null);
                        // and now add it to the list of field stuff we draw.
                        m_drawList.add(new FieldDraw(component, xfm, allianceColor));
                    }
                }
            }

        } catch (IOException | ParseException | ClassCastException | NullPointerException e) {
            // TODO - do something better (like a meaningful error message dialog) for a bad
            // TODO - field description file.
            setDefaultEmptyField();
            resetExtentAndViewDependencies();
            e.printStackTrace();
        }
    }

    static public FieldShape shapeFactory(JSONObject shapeDesc) {
        String type = parseString(shapeDesc, TYPE, null);
        if (null == type) {
            return null;
        }
        switch (type) {
            case TYPE_CIRCLE:
                return new FieldCircle(shapeDesc);
            case TYPE_RECT:
                return new FieldRect(shapeDesc);
            case TYPE_POLYGON:
                return new FieldPolygon(shapeDesc);
            default:
                return null;
        }
    }


    static Color getColor(String colorName, Color defaultColor, Color allianceColor) {
        // OK, the color should be a string that is one of the named colors
        if (null == colorName) {
            return null;
        }
        switch (colorName) {
            case "alliance":
                return allianceColor;
            case "white":
                return Color.WHITE;
            case "red":
                return Color.RED;
            case "blue":
                return Color.BLUE;
            case "yellow":
                return Color.YELLOW;
            case "light-gray":
                return Color.LIGHT_GRAY;
            case "gray":
                return Color.GRAY;
            case "dark-gray":
                return Color.DARK_GRAY;
            case "black":
                return Color.BLACK;
            case "orange":
                return Color.ORANGE;
            case "green":
                return Color.GREEN;
            case "cyan":
                return Color.CYAN;
            case "magenta":
                return Color.MAGENTA;
            case "green-zone":
                return GREEN_ZONE;
            case "yellow-zone":
                return YELLOW_ZONE;
            case "blue-zone":
                return BLUE_ZONE;
            case "purple-zone":
                return PURPLE_ZONE;
            case "red-zone":
                return RED_ZONE;
            default:
                return defaultColor;
        }
    }

    /**
     * Draw the field to the screen.
     *
     * @param g2d     The 2d graphics configuration
     * @param drawXfm The field space to screen space transformation.
     */
    public void draw(@NotNull Graphics2D g2d, @NotNull AffineTransform drawXfm) {
        Stroke oldStroke = g2d.getStroke();
        Color oldColor = g2d.getColor();

        // draw the axis
        g2d.setPaint(Color.ORANGE);
        drawLine(g2d, drawXfm, X_AXIS_START, X_AXIS_END);
        drawLine(g2d, drawXfm, Y_AXIS_START, Y_AXIS_END);

        // draw the field outline as a dotted line
        g2d.setPaint(Color.WHITE);
        drawPolyLine(g2d, drawXfm, FIELD_OUTLINE, m_xfmField, true);

        // now draw the field that was read in from the field data file.
        for (FieldDraw fieldDraw : m_drawList) {
            fieldDraw.draw(g2d, drawXfm);
        }

        g2d.setStroke(oldStroke);
        g2d.setPaint(oldColor);
    }

    /**
     * Test whether a set of points (field relative) is inside the boundary of the field to the specified tolerance.
     * The test fails returns {@code false} at the first point testing closer than tolerance.
     *
     * @param pts       The points to be tested
     * @param tolerance The allowable closeness to the field boundary (in meters)
     * @return {@code false} if any point tests closer than {@code tolerance} to the field
     * boundary, {@code true} otherwise.
     */
    public boolean isInsideField(@NotNull Point2D[] pts, double tolerance) {
        for (Point2D pt : pts) {
            for (Plane2d plane : fieldPlanes) {
                if (!plane.isIn(pt, tolerance)) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     *
     * @param g2d
     * @param drawXfm
     * @param start
     * @param end
     */
    private void drawLine(Graphics2D g2d, AffineTransform drawXfm, Point2D.Double start, Point2D.Double end) {
        Point2D.Double ptStart = (Point2D.Double) drawXfm.transform(start, null);
        Point2D.Double ptEnd = (Point2D.Double) drawXfm.transform(end, null);
        g2d.drawLine((int) ptStart.getX(), (int) ptStart.getY(), (int) ptEnd.getX(), (int) ptEnd.getY());
    }

    /**
     *
     * @param g2d
     * @param drawXfm
     * @param fieldPts
     * @param xfmPts
     * @param closed
     */
    private void drawPolyLine(Graphics2D g2d, AffineTransform drawXfm,
                              Point2D.Double[] fieldPts, Point2D.Double[] xfmPts, boolean closed) {
        drawXfm.transform(fieldPts, 0, xfmPts, 0, fieldPts.length);
        Point2D.Double lastPt = closed ? xfmPts[fieldPts.length - 1] : null;
        for (int n = 0; n < fieldPts.length; n++) {
            Point2D.Double thisPt = xfmPts[n];
            if (null != lastPt) {
                g2d.drawLine((int) thisPt.getX(), (int) thisPt.getY(), (int) lastPt.getX(), (int) lastPt.getY());
            }
            lastPt = thisPt;
        }
    }
}
