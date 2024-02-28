package frc6831.planner.utilities;

import org.a05annex.util.JsonSupport;
import org.jetbrains.annotations.NotNull;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import java.io.FileWriter;

import static org.a05annex.util.JsonSupport.getJSONArray;

public class ReflectOnSpeaker {

    static final String CONTROL_POINTS = "controlPoints";
    static final String FIELD_X = "fieldX";
    static final String FIELD_Y = "fieldY";
    static final String FIELD_HEADING = "fieldHeading";
    static final String TIME = "time";
    static final String LOCATION_DERIVATIVES_EDITED = "derivativesEdited";
    static final String FIELD_dX = "field_dX";
    static final String FIELD_dY = "field_dY";
    static final String HEADING_DERIVATIVE_EDITED = "headingDerivativeEdited";
    static final String FIELD_dHEADING = "field_dHeading";

    static final double SPEAKER_AXIS = -1.4478;

    public static void main(@NotNull final String[] args) {
        System.out.println("*****************************************************************");
        System.out.println("************** Hey, I'm running ReflectOnSpeaker now ************");
        System.out.println("*****************************************************************");
        String infileName = args[0];
        String outfileName = args[1];
        System.out.println(String.format("    Input Path File:  %s", infileName));
        System.out.println(String.format("    Output Path File: %s", outfileName));

        // load the input path:
        System.out.println("Loading path from: " + infileName);
        try {
            JSONObject path = JsonSupport.readJsonFileAsJSONObject("./" + infileName);

            // now mirror the path around x = -1.4478; just need a slight mod on control points:
            // - any X coordinate becomes:  xMirror = SPEAKER_AXIS + (SPEAKER_AXIS - xOriginal)
            // - any i vector becomes:  iMirror = -iOriginal
            // - heading:  headingMirror = -headingO
            JSONArray controlPoints = getJSONArray(path, CONTROL_POINTS);
            for (Object cpObj : controlPoints) {
                JSONObject cpJson = (JSONObject) cpObj;
                cpJson.replace(FIELD_X, SPEAKER_AXIS +
                        (SPEAKER_AXIS - JsonSupport.parseDouble(cpJson, FIELD_X, 0.0)));
                cpJson.replace(FIELD_HEADING, -JsonSupport.parseDouble(cpJson, FIELD_HEADING, 0.0));
                cpJson.replace(FIELD_dX,  -JsonSupport.parseDouble(cpJson, FIELD_dX, 0.0));
                cpJson.replace(FIELD_dHEADING, -JsonSupport.parseDouble(cpJson, FIELD_dHEADING, 0.0));
            }

            FileWriter file = new FileWriter("./" + outfileName);
            file.write(path.toJSONString());
            file.flush();
            file.close();

        } catch (Exception e) {
            e.printStackTrace();
        }

        System.out.flush();
    }
}
