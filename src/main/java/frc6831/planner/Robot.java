package frc6831.planner;

import org.json.simple.JSONObject;
import org.json.simple.parser.ParseException;

import java.io.IOException;

import static frc6831.lib2d.JsonSupport.*;

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

    // ----------------------------------------------------------------------------------------------------
    // Robot Bumper Geometry
    // ----------------------------------------------------------------------------------------------------
    public double getBumperLength() {
        return m_bumperLength;
    }

    public double getBumperWidth() {
        return m_bumperWidth;
    }


    // ----------------------------------------------------------------------------------------------------
    // Loading from a JSON file
    // ----------------------------------------------------------------------------------------------------
    public void loadRobot(String filename) {
        try {
            JSONObject dict = readJsonFile(filename);
            if (null != dict) {
                // Read in the drive geometry
                JSONObject drive = getJSONObject(dict, DRIVE, false);
                if (null != drive) {
                    m_driveLength = parseDouble(drive, LENGTH, m_driveLength);
                    m_driveWidth = parseDouble(drive, WIDTH, m_driveWidth);
                    m_moduleMaxSpeed = parseDouble(drive, MAX_SPEED, m_moduleMaxSpeed);
                }
                // Read in the chassis geometry
                JSONObject chassis = getJSONObject(dict, CHASSIS, false);
                if (null != chassis) {
                    m_chassisLength = parseDouble(chassis, LENGTH, m_chassisLength);
                    m_chassisWidth = parseDouble(chassis, WIDTH, m_chassisWidth);
                }
                // Read in the bumper geometry
                JSONObject bumpers = getJSONObject(dict, BUMPERS, false);
                if (null != bumpers) {
                    m_bumperLength = parseDouble(bumpers, LENGTH, m_bumperLength);
                    m_bumperWidth = parseDouble(bumpers, WIDTH, m_bumperWidth);
                }
            }

        } catch (IOException | ParseException | ClassCastException | NullPointerException e) {
            e.printStackTrace();
        }
    }
}
