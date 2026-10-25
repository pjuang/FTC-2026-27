package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Mecanum drive driven by a Pedro Pathing v3 {@link Follower}. Call {@link #update()} once per loop.
 * All poses are field inches (see {@code auto/FieldPoses}).
 */
public class Drive {
    public final Follower follower;

    public Drive(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    public void setPose(Pose pose) { follower.setPose(pose); }

    public Pose pose() { return follower.pose(); }

    public void update() { follower.update(); }

    public boolean isBusy() { return follower.following(); }

    public void hold(Pose pose) { follower.hold(pose); }

    public void stop() { follower.stop(); }

    /**
     * Drive from where the robot is now through each waypoint, in straight lines. Heading turns
     * smoothly from the current heading to each waypoint's heading along each segment.
     *
     * @param maxSpeed 0..1 fraction of top speed (1 = unlimited)
     */
    public void driveThrough(double maxSpeed, Pose... waypoints) {
        follower.follow(buildPath(pose(), maxSpeed, waypoints));
    }

    public static Path buildPath(Pose from, double maxSpeed, Pose... waypoints) {
        List<Path> segments = new ArrayList<>();
        Pose previous = from;
        for (Pose next : waypoints) {
            segments.add(Paths.line(previous, next).linear(previous, next));
            previous = next;
        }
        Path path = segments.size() == 1 ? segments.get(0)
                : Paths.path(segments.toArray(new Path[0]));
        if (maxSpeed < 1.0) {
            path = path.with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
        }
        return path;
    }

    public void addTelemetry(Telemetry telemetry) {
        Pose p = pose();
        telemetry.addData("Pose (in, deg)", String.format("x %.1f  y %.1f  h %.1f",
                p.x(), p.y(), Math.toDegrees(p.heading())));
        telemetry.addData("Follower", follower.mode());
    }
}
