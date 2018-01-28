using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

[System.Serializable]
public class Problem {

    public float vehicle_L, vehicle_a_max, vehicle_dt, vehicle_omega_max, vehicle_t, vehicle_phi_max, vehicle_v_max;
    public float[] vel_goal, vel_start, pos_goal, pos_start;
    public List<Polygon> obstacles;

    public static Problem Import(string filePath) {
        StreamReader reader = new StreamReader(filePath);
        string json = reader.ReadToEnd();
        reader.Close();
        Problem map = JsonUtility.FromJson<Problem>(json);
        map.read_polygons(json);
        return map;
    }

    private void read_polygons(string jsonString) {
        string[] json = jsonString.Split('\n');
        obstacles = new List<Polygon>();
        for (int i = 0; i < json.Length; i++) {
            if (json[i].Contains("bounding_polygon") || json[i].Contains("obstacle")) {
                Polygon poly = new Polygon(json[i]);
                for (int j = i + 1; j < json.Length - 1; j += 4) {
                    if (json[j].Contains("]") && json[j - 1].Contains("]")) {
                        break;
                    }
                    if (json[j].Contains("[")) {
                        string x_cor = json[j + 1].Trim(' ').Trim(',').Trim(',').Trim('\r').Trim(',');
                        string y_cor = json[j + 2].Trim(' ').Trim(',');
                        double x_coordinate = double.Parse(x_cor, CultureInfo.InvariantCulture);
                        double y_coordinate = double.Parse(y_cor, CultureInfo.InvariantCulture);
                        poly.addCorner(x_coordinate, y_coordinate);
                    }
                }
                obstacles.Add(poly);
            }
        }

    }

}

public enum PolygonType {
    bounding_polygon, obstacle
}

public class Polygon {
    public PolygonType type;
    public List<float[]> corners;

    public Polygon(string type) {
        corners = new List<float[]>();
        if (type.Contains(PolygonType.bounding_polygon.ToString())) {
            this.type = PolygonType.bounding_polygon;
        } else {
            this.type = PolygonType.obstacle;
        }
    }

    public void addCorner(double x, double y) {
        float[] corner = { (float)x, (float)y };
        corners.Add(corner);
    }
}