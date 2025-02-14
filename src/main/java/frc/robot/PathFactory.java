package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
 
import java.io.IOException;
import java.util.HashMap;

import org.javatuples.Pair;
import org.json.simple.parser.ParseException;

public class PathFactory {

    private static final HashMap<Pair<Pose2d,Pose2d>, String> pathMap;

    static {
        pathMap = new HashMap<>();
            pathMap.put(Pair.with(FieldConstants.ROBOT_START_1, FieldConstants.REEF_F), "Start1 to ReefF");
            pathMap.put(Pair.with(FieldConstants.ROBOT_START_2, FieldConstants.REEF_H), "Start2 to ReefH");
            pathMap.put(Pair.with(FieldConstants.ROBOT_START_3, FieldConstants.REEF_I), "Start3 to ReefI");
            pathMap.put(Pair.with(FieldConstants.ROBOT_START_3, FieldConstants.REEF_J), "Start3 to ReefJ");
      
            pathMap.put(Pair.with(FieldConstants.REEF_A, FieldConstants.SOURCE_2_IN), "ReefA to Source2In");
            pathMap.put(Pair.with(FieldConstants.REEF_B, FieldConstants.SOURCE_1_IN), "ReefB to Source1In");
            pathMap.put(Pair.with(FieldConstants.REEF_E, FieldConstants.SOURCE_1_IN), "ReefE to Source1In");
            pathMap.put(Pair.with(FieldConstants.REEF_F, FieldConstants.SOURCE_1_IN), "ReefF to Source1In");

            pathMap.put(Pair.with(FieldConstants.REEF_H, FieldConstants.SOURCE_2_CENTER), "ReefH to Source2Center");
            pathMap.put(Pair.with(FieldConstants.REEF_H, FieldConstants.SOURCE_2_OUT), "ReefH to Source2Out");
            pathMap.put(Pair.with(FieldConstants.REEF_I, FieldConstants.SOURCE_2_IN), "ReefI to Source2In");
            pathMap.put(Pair.with(FieldConstants.REEF_I, FieldConstants.SOURCE_2_OUT), "ReefI to Source2Out");
            pathMap.put(Pair.with(FieldConstants.REEF_I, FieldConstants.SOURCE_2_CENTER), "ReefI to Source2Center");
            pathMap.put(Pair.with(FieldConstants.REEF_J, FieldConstants.SOURCE_2_IN), "ReefJ to Source2In");
            pathMap.put(Pair.with(FieldConstants.REEF_J, FieldConstants.SOURCE_2_OUT), "ReefJ to Source2Out");
            pathMap.put(Pair.with(FieldConstants.REEF_J, FieldConstants.SOURCE_2_CENTER), "ReefJ to Source2Center");
            pathMap.put(Pair.with(FieldConstants.REEF_K, FieldConstants.SOURCE_2_IN), "ReefK to Source2In");
            pathMap.put(Pair.with(FieldConstants.REEF_K, FieldConstants.SOURCE_2_OUT), "ReefK to Source2Out");
            pathMap.put(Pair.with(FieldConstants.REEF_K, FieldConstants.SOURCE_2_CENTER), "ReefK to Source2Center");

            pathMap.put(Pair.with(FieldConstants.REEF_L, FieldConstants.SOURCE_2_IN), "ReefL to Source2In");
            pathMap.put(Pair.with(FieldConstants.REEF_L, FieldConstants.SOURCE_2_OUT), "ReefL to Source2Out");
            pathMap.put(Pair.with(FieldConstants.REEF_L, FieldConstants.SOURCE_2_CENTER), "ReefL to Source2Center");


            pathMap.put(Pair.with(FieldConstants.SOURCE_1_IN, FieldConstants.REEF_B), "Source1In to ReefB");
            pathMap.put(Pair.with(FieldConstants.SOURCE_1_IN, FieldConstants.REEF_C), "Source1In to ReefC");
            pathMap.put(Pair.with(FieldConstants.SOURCE_1_IN, FieldConstants.REEF_D), "Source1In to ReefD");

            pathMap.put(Pair.with(FieldConstants.SOURCE_2_IN, FieldConstants.REEF_A), "Source2In to ReefA");
            pathMap.put(Pair.with(FieldConstants.SOURCE_2_IN, FieldConstants.REEF_B), "Source2In to ReefB");
            pathMap.put(Pair.with(FieldConstants.SOURCE_2_IN, FieldConstants.REEF_K), "Source2In to ReefK");
            pathMap.put(Pair.with(FieldConstants.SOURCE_2_IN, FieldConstants.REEF_L), "Source2In to ReefL");

            pathMap.put(Pair.with(FieldConstants.SOURCE_2_CENTER, FieldConstants.REEF_A), "Source2Center to ReefA");
            pathMap.put(Pair.with(FieldConstants.SOURCE_2_CENTER, FieldConstants.REEF_J), "Source2Center to ReefJ");
            pathMap.put(Pair.with(FieldConstants.SOURCE_2_CENTER, FieldConstants.REEF_K), "Source2Center to ReefK");
            pathMap.put(Pair.with(FieldConstants.SOURCE_2_CENTER, FieldConstants.REEF_L), "Source2Center to ReefL");

            pathMap.put(Pair.with(FieldConstants.SOURCE_2_OUT, FieldConstants.REEF_A), "Source2Out to ReefA");
            pathMap.put(Pair.with(FieldConstants.SOURCE_2_OUT, FieldConstants.REEF_K), "Source2Out to ReefK");
            pathMap.put(Pair.with(FieldConstants.SOURCE_2_OUT, FieldConstants.REEF_L), "Source2Out to ReefL");



    }

    public static PathPlannerPath getPath(Pose2d start, Pose2d end) {
        return getPath(start, end, false);
    }
    
    public static PathPlannerPath getPath(Pose2d start, Pose2d end, boolean mirrorPath) {
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile(pathMap.get(Pair.with(start, end)));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        } 
        return mirrorPath ? path.mirrorPath() : path;
    }

}
