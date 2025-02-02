package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

// FYI - This is a test class that is not used in the robot code
// It is used to test the PathPlannerPath class, and the mirroring functionality
// to ensure that the path is mirrored correctly

// To run, add this to the launch line in VSCode terminal:
// '-Djava.library.path=C:\Users\Jake Hendrickson\gitdev\com.github\ligerbots\ReefScape2025\build\jni\release'


public class TestPathMirror {

    public static void main(String[] args) {
        try {
            PathPlannerPath testPath1 = PathPlannerPath.fromPathFile("Start1 to ReefF");
            PathPlannerPath testPath2 = PathPlannerPath.fromPathFile("Start1 to ReefF-Copy");

            System.out.println(testPath1);
            System.out.println(testPath2);

            System.out.println(testPath1.equals(testPath2));

            testPath1.mirrorPath();
        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
