package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

import java.io.IOException;
import java.util.HashMap;

import org.json.simple.parser.ParseException;

public class PathFactory {

    private static final HashMap<PointPair, String> pathMap;

    static {
        pathMap = new HashMap<>();
            pathMap.put(new PointPair(FieldConstants.ROBOT_START_1, FieldConstants.REEF_F), "Start1 to ReefF");
            pathMap.put(new PointPair(FieldConstants.ROBOT_START_2, FieldConstants.REEF_H), "Start2 to ReefH");
            pathMap.put(new PointPair(FieldConstants.ROBOT_START_3, FieldConstants.REEF_J), "Start3 to ReefJ");

            pathMap.put(new PointPair(FieldConstants.REEF_J, FieldConstants.SOURCE_2_OUT), "ReefJ to Source2Out");
            pathMap.put(new PointPair(FieldConstants.REEF_K, FieldConstants.SOURCE_2_OUT), "ReefK to Source2Out");
            pathMap.put(new PointPair(FieldConstants.REEF_L, FieldConstants.SOURCE_2_OUT), "ReefL to Source2Out");

            pathMap.put(new PointPair(FieldConstants.SOURCE_2_OUT, FieldConstants.REEF_A), "Source2Out to ReefA");
            pathMap.put(new PointPair(FieldConstants.SOURCE_2_OUT, FieldConstants.REEF_J), "Source2Out to ReefJ");
            pathMap.put(new PointPair(FieldConstants.SOURCE_2_OUT, FieldConstants.REEF_K), "Source2Out to ReefK");
            pathMap.put(new PointPair(FieldConstants.SOURCE_2_OUT, FieldConstants.REEF_L), "Source2Out to ReefL");

    }

    public static PathPlannerPath getPath(Pose2d start, Pose2d end) {
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile(pathMap.get(new PointPair(start, end)));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        } 
        return path;
    }


}
