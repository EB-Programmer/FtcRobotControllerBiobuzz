package org.firstinspires.ftc.teamcode.routes;

import com.pedropathing.paths.Path;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

/**
 * An autonomous route: a set of paths pasted from the online path visualizer (see package-info.java)
 * plus what the robot does around them. A subclass passes its pasted paths object to the
 * constructor and overrides atStart() and/or afterPath() for anything beyond driving; both do
 * nothing by default.
 *
 * <pre>
 * public class FarGoalRoute extends Route {
 *     public FarGoalRoute() {
 *         super(new FarGoalPath());
 *     }
 *
 *     &#64;Override
 *     public void afterPath(int pathNumber, RouteActions robot) {
 *         switch (pathNumber) {
 *             case 1: robot.intakeOn(); break;  // runs while path 2 drives
 *             case 2: robot.waitMillis(1500); robot.intakeOff(); break;
 *             case 4: robot.waitMillis(500); break;
 *         }
 *     }
 * }
 * </pre>
 *
 * Do everything through the RouteActions helpers rather than sleep() or loops of your own: the
 * helpers keep the robot holding its position and the telemetry updating, and they return as
 * soon as Stop is pressed. If an action throws, BiobuzzAuton flags it in the end-of-run summary
 * and carries on with the next path.
 */
public abstract class Route {
    private final Object paths;

    protected Route(Object paths) {
        this.paths = paths;
    }

    /**
     * Collects path1(), path2(), ... from the pasted paths object by the number in each method's
     * name, stopping at the first missing one -- that's the naming convention the online path
     * visualizer uses, so any pasted route runs without its path count being wired in by hand.
     *
     * @throws IllegalStateException naming the method, if one of them can't be called or throws
     */
    public final List<Path> buildPaths() {
        List<Path> result = new ArrayList<>();
        for (int i = 1; ; i++) {
            Method method;
            try {
                method = paths.getClass().getMethod("path" + i);
            } catch (NoSuchMethodException e) {
                break;
            }
            try {
                result.add((Path) method.invoke(paths));
            } catch (InvocationTargetException e) {
                throw new IllegalStateException(method.getName() + "() threw " + e.getCause(),
                        e.getCause());
            } catch (IllegalAccessException e) {
                throw new IllegalStateException(
                        "can't call " + method.getName() + "() -- is the pasted class public?", e);
            }
        }
        return result;
    }

    /** Runs once before path 1 starts, while the robot holds its start pose. */
    public void atStart(RouteActions robot) {
    }

    /**
     * Runs after each path ends, while the robot holds that path's end pose. pathNumber matches
     * the pasted method's name: 1 after path1(), 2 after path2(), and so on.
     */
    public void afterPath(int pathNumber, RouteActions robot) {
    }
}
