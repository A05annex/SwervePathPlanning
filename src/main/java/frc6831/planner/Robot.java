package frc6831.planner;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Dictionary;

/**
 * This class is a description of the robot. There is a default programmed description, but it is expected that
 * the actual description will be read from a data file
 */
public class Robot {

    private static final String DRIVE = "drive";
    private static final String CHASSIS = "chassis";
    private static final String BUMPERS = "bumpers";
    private static final String LENGTH = "length";
    private static final String WIDTH = "width";
    private static final String MAX_SPEED = "maxSpeed";

    private double m_driveLength = 0.7;
    private double m_driveWidth = 0.3;

    private double m_chassisLength = 0.9;
    private double m_chassisWidth = 0.5;

    private double m_bumperLength = 1.1;
    private double m_bumperWidth = 0.7;

    private double m_moduleMaxSpeed = 3.0;

    // ----------------------------------------------------------------------------------------------------
    // Swerve Drive Geometry
    // ----------------------------------------------------------------------------------------------------
    public double getDriveLength() {
        return m_driveLength;
    }

    public double getDriveWidth() {
        return m_driveWidth;
    }

    public double getDriveRadius() {
        return 0.5 * Math.sqrt((m_driveLength * m_driveLength) + (m_driveWidth * m_driveWidth));
    }

    public double getMaxModuleSpeed() {
        return m_moduleMaxSpeed;
    }

    public double getMaxRotationalSpeed() {
        return m_moduleMaxSpeed / getDriveRadius();

    }
    // ----------------------------------------------------------------------------------------------------
    // Robot Chassis Geometry
    // ----------------------------------------------------------------------------------------------------
    public double getChassisLength() {
        return m_chassisLength;
    }

    public double getChassisWidth() {
        return m_chassisWidth;
    }

    public double getBumperLength() {
        return m_bumperLength;
    }

    public double getBumperWidth() {
        return m_bumperWidth;
    }


    public void loadRobot(String filename) {
        //JSON parser object to parse read file
        JSONParser jsonParser = new JSONParser();

        try (FileReader reader = new FileReader(filename)) {
            //Read JSON file
            Object obj = jsonParser.parse(reader);
            if (obj.getClass() == JSONObject.class) {
                JSONObject dict = (JSONObject) obj;
                // Read in the drive geometry
                Object drive = dict.get(DRIVE);
                if ((null != drive) && (drive.getClass() == JSONObject.class)) {
                    m_driveLength = parseDouble((JSONObject) drive, LENGTH, m_driveLength);
                    m_driveWidth = parseDouble((JSONObject) drive, WIDTH, m_driveWidth);
                    m_moduleMaxSpeed = parseDouble((JSONObject) drive, MAX_SPEED, m_moduleMaxSpeed);
                }
                // Read in the chassis geometry
                Object chassis = dict.get(CHASSIS);
                if ((null != chassis) && (chassis.getClass() == JSONObject.class)) {
                    m_chassisLength = parseDouble((JSONObject) chassis, LENGTH, m_chassisLength);
                    m_chassisWidth = parseDouble((JSONObject) chassis, WIDTH, m_chassisWidth);
                }
                // Read in the bumper geometry
                Object bumpers = dict.get(BUMPERS);
                if ((null != bumpers) && (bumpers.getClass() == JSONObject.class)) {
                    m_bumperLength = parseDouble((JSONObject) bumpers, LENGTH, m_bumperLength);
                    m_bumperWidth = parseDouble((JSONObject) bumpers, WIDTH, m_bumperWidth);
                }

            }
            double x = 1.0;

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ParseException e) {
            e.printStackTrace();
        }
    }

    private double parseDouble(JSONObject dict, String key, double defaultValue) {
        double value = defaultValue;
        Object valueObj = dict.get(key);
        if (null != valueObj) {
            value = (double) valueObj;
        }
        return value;
    }
}
