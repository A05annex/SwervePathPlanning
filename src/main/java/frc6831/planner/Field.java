package frc6831.planner;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import static frc6831.lib2d.JsonSupport.*;

/**
 * This class maintains and draws the field we will be plotting the path on. Note that the default
 * with no loaded field data is to draw the axes and a dotted outline of the field.
 */
public class Field {

    // -------------------------------------------------------------------------------------------
    // Parsing the field file
    private static final String COMPONENTS = "components";
    private static final String FIELD = "field";

    private static final String NAME = "name";
    private static final String LINE_COLOR = "lineColor";
    private static final String FILL_COLOR = "fillColor";
    private static final String SHAPES = "shapes";

    private static final String TYPE = "type";

    private static final String TYPE_CIRCLE = "circle";
    private static final String CIRCLE_CENTER = "center";
    private static final String CIRCLE_RADIUS = "radius";

    private static final String TYPE_RECT = "rect";
    private static final String RECT_LOWER_LEFT = "lower left";
    private static final String RECT_UPPER_RIGHT = "upper right";

    private static final String COMPONENT = "component";
    private static final String ALLIANCE = "alliance";
    private static final String TRANSLATE = "translate";
    private static final String ROTATE = "rotate";

    // -------------------------------------------------------------------------------------------
    // The default standard field axes and outline
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

    private MinMax m_minMax = new MinMax(X_AXIS_MIN, Y_AXIS_MIN, X_AXIS_MAX, Y_AXIS_MAX);
    private HashMap<String, FieldComponent> m_components = new HashMap();
    private ArrayList<FieldDraw> m_drawList = new ArrayList();

    // -------------------------------------------------------------------------------------------

    public class MinMax {
        protected double m_minX;
        protected double m_minY;
        protected double m_maxX;
        protected double m_maxY;

        public MinMax(double minX, double minY, double maxX, double maxY) {
            m_minX = minX;
            m_minY = minY;
            m_maxX = maxX;
            m_maxY = maxY;
        }

        public double getMinX() {
            return m_minX;
        }

        public double getMinY() {
            return m_minY;
        }

        public double getMaxX() {
            return m_maxX;
        }

        public double getMaxY() {
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

        Point2D m_center;
        double m_radius;


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
                g2d.drawOval((int)(ptCenter.getX() - scaledRadius), (int) (ptCenter.getY() - scaledRadius),
                        (int)(2.0 * scaledRadius), (int)(2.0 * scaledRadius));
            }
            if (null != fill) {
                g2d.setPaint(fill);
                g2d.fillOval((int)(ptCenter.getX() - scaledRadius), (int) (ptCenter.getY() - scaledRadius),
                        (int)(2.0 * scaledRadius), (int)(2.0 * scaledRadius));
            }

        }
    }

    private static class FieldRect extends FieldShape {
        Point2D m_LL;
        Point2D m_UR;

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
                x = (int)ptLL.x;
                width = (int)(ptUR.x - ptLL.x);
            } else {
                x = (int)ptUR.x;
                width = (int)(ptLL.x - ptUR.x);

            }
            if (ptLL.y < ptUR.y) {
                y = (int)ptLL.y;
                height = (int)(ptUR.y - ptLL.y);
            } else {
                y = (int)ptUR.y;
                height = (int)(ptLL.y - ptUR.y);

            }
            if (null != outline) {
                g2d.setPaint(outline);
                g2d.drawRect(x,y,width,height);
            }
            if (null != fill) {
                g2d.setPaint(fill);
                g2d.fillRect(x,y,width,height);
            }

        }
    }

    // -------------------------------------------------------------------------------------------
    // Components that may appear on the field multiple times, and optionally in alliance colors
    // -------------------------------------------------------------------------------------------
    private class FieldComponent {
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
                FieldShape fieldShape = shapeFactory((JSONObject)shape);
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
                        (null == m_outlineColor) ? null : getColor(m_outlineColor,null,allianceColor),
                        (null == m_fillColor) ? null : getColor(m_fillColor,null,allianceColor));
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
    class FieldDraw {
        FieldComponent m_component;
        AffineTransform m_xfm;
        Color m_allianceColor;

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

    // ----------------------------------------------------------------------------------------------------
    public MinMax getMinMax() {
        return m_minMax;
    }

    // ----------------------------------------------------------------------------------------------------
    // Loading from a JSON file
    // ----------------------------------------------------------------------------------------------------
    public void loadField(String filename) {
        try {
            JSONObject dict = readJsonFile(filename);
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
                            xfm.translate(translate.getX(),translate.getY());
                        }
                        // and set the alliance color (if there is one)
                        String colorName = parseString(drawDesc, ALLIANCE, null);
                        Color allianceColor = (null == colorName) ? null : getColor(colorName,null,null);
                        // and now add it to the list of field stuff we draw.
                        m_drawList.add(new FieldDraw(component,xfm,allianceColor));
                    }
                }
            }

        } catch (IOException | ParseException | ClassCastException | NullPointerException e) {
            e.printStackTrace();
        }
    }

    static public FieldShape shapeFactory(JSONObject shapeDesc) {
        String type = parseString(shapeDesc, TYPE, null);
        FieldShape fieldShape = null;
        switch (type) {
            case TYPE_CIRCLE:
                return new FieldCircle(shapeDesc);
            case TYPE_RECT:
                return new FieldRect(shapeDesc);
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
            default:
                return defaultColor;
        }
    }

    public void draw(Graphics2D g2d, AffineTransform drawXfm) {
        Stroke oldStroke = g2d.getStroke();

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
    }

    /**
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
