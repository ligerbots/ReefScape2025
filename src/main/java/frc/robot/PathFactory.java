package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
 
import java.io.IOException;
import java.util.HashMap;

import org.javatuples.Pair;
import org.json.simple.parser.ParseException;

public class PathFactory {

    private static final HashMap<Pose2d,String> pointNames;
    static {
        pointNames = new HashMap<>();
        pointNames.put(FieldConstants.REEF_A, "ReefA");
        pointNames.put(FieldConstants.REEF_B, "ReefB");
        pointNames.put(FieldConstants.REEF_C, "ReefC");
        pointNames.put(FieldConstants.REEF_D, "ReefD");
        pointNames.put(FieldConstants.REEF_E, "ReefE");
        pointNames.put(FieldConstants.REEF_F, "ReefF");
        pointNames.put(FieldConstants.REEF_G, "ReefG");
        pointNames.put(FieldConstants.REEF_H, "ReefH");
        pointNames.put(FieldConstants.REEF_I, "ReefI");
        pointNames.put(FieldConstants.REEF_J, "ReefJ");
        pointNames.put(FieldConstants.REEF_K, "ReefK");
        pointNames.put(FieldConstants.REEF_L, "ReefL");

        pointNames.put(FieldConstants.SOURCE_2_IN, "Source2In");
        pointNames.put(FieldConstants.SOURCE_2_CENTER, "Source2Center");
        pointNames.put(FieldConstants.SOURCE_2_OUT, "Source2Out");

        pointNames.put(FieldConstants.ROBOT_START_1, "Start1");
        pointNames.put(FieldConstants.ROBOT_START_2, "Start2");
        pointNames.put(FieldConstants.ROBOT_START_3, "Start3");        

    }


    public static PathPlannerPath getPath(Pose2d start, Pose2d end) {
        return getPath(start, end, false);
    }
    
    public static PathPlannerPath getPath(Pose2d start, Pose2d end, boolean mirrorPath) {
        PathPlannerPath path = null;
        StringBuilder sb = new StringBuilder(40);
        String pathName =  sb.append(pointNames.get(start)).append(" to ").append(pointNames.get(end)).toString();
        
        // String pathName = pointNames.get(start) + " to " + pointNames.get(end);
        try {
            // path = PathPlannerPath.fromPathFile(pathMap.get(Pair.with(start, end)));
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            System.out.println(pathName);
            System.out.println(Pair.with(start, end));

            e.printStackTrace();
        } 
        return mirrorPath ? path.mirrorPath() : path;
    }

}
