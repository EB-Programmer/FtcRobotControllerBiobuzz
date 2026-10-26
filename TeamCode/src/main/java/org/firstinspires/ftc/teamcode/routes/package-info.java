/**
 * Autonomous routes, two classes each, side by side: the route's paths, pasted from the online
 * path visualizer (https://visualizer.pedropathing.com/), and a Route that pairs those paths with
 * what the robot does between them -- for example UpCellPath and UpCellRoute. A Path class holds
 * only the pasted code, so a route can be re-pasted after changing it in the visualizer; anything
 * else about the route goes in its Route class.
 *
 * To add a route:
 * <ol>
 *   <li>Create a new file here named after the route, e.g. {@code FarGoalPath.java}.</li>
 *   <li>Add the line {@code package org.firstinspires.ftc.teamcode.routes;} at the top, then
 *       paste the visualizer's generated code below it.</li>
 *   <li>Rename the pasted {@code public class Paths} to match the file name
 *       ({@code public class FarGoalPath}).</li>
 *   <li>Create {@code FarGoalRoute.java} next to it, whose constructor calls
 *       {@code super(new FarGoalPath())} -- see UpCellRoute.</li>
 *   <li>Add {@code new FarGoalRoute()} to the {@code routes} list at the top of BiobuzzAuton so
 *       it shows up in the init menu.</li>
 * </ol>
 *
 * BiobuzzAuton runs a route's path1(), path2(), ... in order and stops at the first missing
 * number, so it doesn't matter how many paths a route has. The visualizer names each generated
 * method after its path's name, so in the visualizer leave each path's name blank or set to
 * "Path N" matching its place in the route: a path named anything else gets a method that never
 * runs.
 */
package org.firstinspires.ftc.teamcode.routes;
