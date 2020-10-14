package frc6831.lib2d;

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

    static public double parseDouble(JSONObject dict, String key, double defaultValue) {
        double value = defaultValue;
        Object valueObj = dict.get(key);
        if (null != valueObj) {
            value = (double) valueObj;
        }
        return value;
    }
    static public boolean parseBoolean(JSONObject dict, String key, boolean defaultValue) {
        boolean value = defaultValue;
        Object valueObj = dict.get(key);
        if (null != valueObj) {
            value = (boolean) valueObj;
        }
        return value;
    }

    static public Point2D parsePoint(JSONObject dict, String key) {
        Object valueObj = dict.get(key);
        if ((null != valueObj) && (valueObj.getClass() == JSONArray.class)) {
            return parsePoint((JSONArray) valueObj);
        }
        return null;
    }

    static public Point2D parsePoint(JSONArray coordList) {
        return new Point2D.Double((double) coordList.get(0), (double) coordList.get(1));
    }

    static public String parseString(JSONObject dict, String key, String defaultValue) {
        String value = defaultValue;
        Object valueObj = dict.get(key);
        if (null != valueObj) {
            value = (String) valueObj;
        }
        return value;
    }

    static public JSONArray getJSONArray(JSONObject dict, String key) {
        return getJSONArray(dict, key, true);
    }

    static public JSONArray getJSONArray(JSONObject dict, String key, boolean notNull) {
        Object obj = dict.get(key);
        if (notNull && (null == obj)) {
            throw new NullPointerException(String.format("No value for key '%s'", key));
        }
        return (JSONArray) obj;
    }

    static public JSONObject getJSONObject(JSONObject dict, String key) {
        return getJSONObject(dict, key, true);
    }

    static public JSONObject getJSONObject(JSONObject dict, String key, boolean notNull) {
        Object obj = dict.get(key);
        if (notNull && (null == obj)) {
            throw new NullPointerException(String.format("No value for key '%s'", key));
        }
        return (JSONObject) obj;
    }

    static public JSONObject readJsonFile(String filename) throws
            IOException, ParseException {
        JSONParser jsonParser = new JSONParser();
        FileReader reader = new FileReader(filename);
        //Read JSON file
        Object obj = jsonParser.parse(reader);
        return (JSONObject) obj;
    }
}
