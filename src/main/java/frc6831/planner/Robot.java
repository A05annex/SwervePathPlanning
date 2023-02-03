package frc6831.planner;

import org.a05annex.util.Utl;
import org.json.simple.JSONObject;
import org.json.simple.parser.ParseException;

import java.io.IOException;

import static org.a05annex.util.JsonSupport.*;

/**
 * This class is a description of the robot. There is a default programmed description, but it is expected that
 * the actual description will be read from a data file
 */
@SuppressWarnings("HungarianNotationMemberVariables")
public class Robot {

    private static final String DRIVE = "drive";
    private static final String CHASSIS = "chassis";
    private static final String BUMPERS = "bumpers";
    private static final String LENGTH = "length";
    private static final String WIDTH = "width";
    private static final String MAX_SPEED = "maxSpeed";

    private double m_driveLength = 0.7;
    private double m_driveWidth = 0.3;
    private double m_driveDiagonal = Utl.length(m_driveLength, m_driveWidth);
    private double m_driveRadius = m_driveDiagonal / 2.0;

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
        return m_driveRadius;
    }

    public double getMaxModuleSpeed() {
        return m_moduleMaxSpeed;
    }

    public double getMaxRotationalSpeed() {
        return m_moduleMaxSpeed / getDriveRadius();

    }

    /**
     * Test whether the robot can actually achieve the module speeds required to follow
     * the path forward, strafe, and rotation speeds.
     * @param forward The forward speed, meters/sec.
     * @param strafe The strafe speed, meters/sec.
     * @param rotation The rotation, radians/sec
     * @return {@code true} if the robot can achieve these speeds, {@code false} if
     * these speeds exceed the capability of the robot.
     */
    public boolean canRobotAchieve(double forward, double strafe, double rotation)
    {
        // calculate a, b, c and d variables
        double a = strafe - (rotation * (m_driveLength / m_driveDiagonal));
        double b = strafe + (rotation * (m_driveLength / m_driveDiagonal));
        double c = forward - (rotation * (m_driveWidth / m_driveDiagonal));
        double d = forward + (rotation * (m_driveWidth / m_driveDiagonal));

        // calculate module speeds, if they are all less than the max, we are good
        return (Utl.length(b, c) <= m_moduleMaxSpeed) &&    // right front
                (Utl.length(b, d) <= m_moduleMaxSpeed) &&   // left front
                (Utl.length(a, d) <= m_moduleMaxSpeed) &&   // left rear
                (Utl.length(a, c) <= m_moduleMaxSpeed) ;    // right rear
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
            JSONObject dict = readJsonFileAsJSONObject(filename);
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
                m_driveDiagonal = Utl.length(m_driveLength, m_driveWidth);
                m_driveRadius = m_driveDiagonal / 2.0;
            }


        } catch (IOException | ParseException | ClassCastException | NullPointerException e) {
            e.printStackTrace();
        }
    }
}
