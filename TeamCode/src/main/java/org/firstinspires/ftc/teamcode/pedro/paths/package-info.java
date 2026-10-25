/**
 * The paths for each autonomous route, one class per route, pasted from the online path
 * visualizer (https://visualizer.pedropathing.com/). These files hold only the pasted code, so a
 * route can be re-pasted after changing it in the visualizer; what the robot does between paths
 * lives in the matching Route class in the routes package.
 *
 * To add a route:
 * <ol>
 *   <li>Create a new file here named after the route, e.g. {@code FarGoalPath.java}.</li>
 *   <li>Add the line {@code package org.firstinspires.ftc.teamcode.pedro.paths;} at the top,
 *       then paste the visualizer's generated code below it.</li>
 *   <li>Rename the pasted {@code public class Paths} to match the file name
 *       ({@code public class FarGoalPath}).</li>
 *   <li>Create a matching Route in the routes package, e.g. {@code FarGoalRoute}, whose
 *       constructor calls {@code super(new FarGoalPath())} -- see UpCellRoute.</li>
 *   <li>Add {@code FarGoalRoute.class} to {@code BiobuzzAuton.ROUTES} so it shows up in the
 *       init menu.</li>
 * </ol>
 *
 * BiobuzzAuton runs a route's path1(), path2(), ... in order and stops at the first missing
 * number, so it doesn't matter how many paths a route has.
 */
package org.firstinspires.ftc.teamcode.pedro.paths;
