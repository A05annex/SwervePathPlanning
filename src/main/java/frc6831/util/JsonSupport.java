package frc6831.util;

import org.jetbrains.annotations.NotNull;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.awt.geom.Point2D;
import java.io.FileReader;
import java.io.IOException;

/**
 * This class provides utility methods for standard data types for robotics with well known default
 * and failure behaviour.
 */
public class JsonSupport {

    /**
     * Parse the double value for an optional key in a JSON dictionary. If the key does not exist, the
     * default will be returned.
     *
     * @param dict The JSON representation of a dictionary
     * @param key The key for the value to be obtained.
     * @param defaultValue The default if the key has not been specified.
     * @return Returns the value for the key.
     */
    static public double parseDouble(@NotNull JSONObject dict, @NotNull String key, double defaultValue) {
        double value = defaultValue;
        Object valueObj = dict.get(key);
        if (null != valueObj) {
            value = (double) valueObj;
        }
        return value;
    }

    /**
     * Parse the boolean value for an optional key in a JSON dictionary. If the key does not exist, the
     * default will be returned.
     *
     * @param dict The JSON representation of a dictionary
     * @param key The key for the value to be obtained.
     * @param defaultValue The default if the key has not been specified.
     * @return Returns the value for the key.
     */
    static public boolean parseBoolean(JSONObject dict, String key, boolean defaultValue) {
        boolean value = defaultValue;
        Object valueObj = dict.get(key);
        if (null != valueObj) {
            value = (boolean) valueObj;
        }
        return value;
    }

    /**
     * Parse the point value for an optional key in a JSON dictionary. If the key does not exist,
     * <tt>null</tt> will be returned.
     *
     * @param dict The JSON representation of a dictionary
     * @param key The key for the value to be obtained.
     * @return The value of the parse point if the <tt>key</tt> is specified, <tt>null</tt> otherwise.
     */
    static public Point2D parsePoint(JSONObject dict, String key) {
        Object valueObj = dict.get(key);
        if ((null != valueObj) && (valueObj.getClass() == JSONArray.class)) {
            return parsePoint((JSONArray) valueObj);
        }
        return null;
    }

    /**
     *
     * @param coordList
     * @return
     */
    static public Point2D parsePoint(JSONArray coordList) {
        return new Point2D.Double((double) coordList.get(0), (double) coordList.get(1));
    }

    /**
     *
     * @param dict
     * @param key
     * @param defaultValue
     * @return
     */
    static public String parseString(JSONObject dict, String key, String defaultValue) {
        String value = defaultValue;
        Object valueObj = dict.get(key);
        if (null != valueObj) {
            value = (String) valueObj;
        }
        return value;
    }

    /**
     *
     * @param dict
     * @param key
     * @return
     */
    static public JSONArray getJSONArray(JSONObject dict, String key) {
        return getJSONArray(dict, key, true);
    }

    /**
     *
     * @param dict
     * @param key
     * @param notNull
     * @return
     */
    static public JSONArray getJSONArray(JSONObject dict, String key, boolean notNull) {
        Object obj = dict.get(key);
        if (notNull && (null == obj)) {
            throw new NullPointerException(String.format("No value for key '%s'", key));
        }
        return (JSONArray) obj;
    }

    /**
     *
     * @param dict
     * @param key
     * @return
     */
    static public JSONObject getJSONObject(JSONObject dict, String key) {
        return getJSONObject(dict, key, true);
    }

    /**
     *
     * @param dict
     * @param key
     * @param notNull
     * @return
     */
    static public JSONObject getJSONObject(JSONObject dict, String key, boolean notNull) {
        Object obj = dict.get(key);
        if (notNull && (null == obj)) {
            throw new NullPointerException(String.format("No value for key '%s'", key));
        }
        return (JSONObject) obj;
    }

    /**
     *
     * @param filename
     * @return
     * @throws IOException
     * @throws ParseException
     */
    static public JSONObject readJsonFile(String filename) throws
            IOException, ParseException {
        JSONParser jsonParser = new JSONParser();
        FileReader reader = new FileReader(filename);
        //Read JSON file
        Object obj = jsonParser.parse(reader);
        return (JSONObject) obj;
    }
}
