package org.firstinspires.ftc.teamcode.Season.SensorStuff;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

//Processes AprilTagData
//Old code, doesn't work anymore

public class AprilTagExtractor {
    private static final String LIMELIGHT_URL = "http://172.29.0.1:5807/json";
    // ^ Change this IP to your Limelight's address

    public static TagData getAprilTagData() {
        try {
            // Open connection
            URL url = new URL(LIMELIGHT_URL);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("GET");

            // Read response
            BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()));
            StringBuilder response = new StringBuilder();
            String line;
            while ((line = in.readLine()) != null) {
                response.append(line);
            }
            in.close();

            // Parse JSON
            JSONObject data = new JSONObject(response.toString());
            if (data.has("Results")) {
                JSONObject results = data.getJSONObject("Results");

                if (results.has("FiducialResults") && results.getJSONArray("FiducialResults").length() > 0) {
                    JSONObject tag = results.getJSONArray("FiducialResults").getJSONObject(0);

                    int fID = tag.getInt("fID");
                    double tx = tag.getDouble("tx");
                    double ty = tag.getDouble("ty");
                    double tz = tag.getDouble("tz");

                    return new TagData(fID, tx, ty, tz, true);
                }
            }
        } catch (Exception e) {
            // Ignore errors → will just return "invalid"
        }

        return new TagData(-1, 0, 0, 0, false); // invalid/no tag
    }
}
