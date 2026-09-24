import {
    makeWaypoint,
    reflectWaypoints,
    makeEventMarker,
    makeConstraint,
    toExpr,
    AllLocationProperties
} from "../types";

const HUB_X = 4.63710355758667;
const HUB_Y = 4.025846004486084;

/*
 * SOTM route based on the waypoint layout from the SOTM.traj you provided.
 *
 * Important:
 * - The waypoint coordinates/poses below are copied from the PARAMS section
 *   of that trajectory, not from the generated samples.
 * - There are two shoot-on-the-move windows:
 *      waypoint 6 -> 8
 *      waypoint 15 -> 17
 * - Both windows are capped at 1.5 m/s.
 * - Shooting is controlled with event markers, not splitCommands.
 * - PointAt toward the Hub should be added to those same two ranges once the
 *   project's local PointAt ConstraintData shape is confirmed.
 */

export const allLocationProperties: AllLocationProperties = {
    start: {
        leftWaypoints: [
            makeWaypoint(4.4603095054626465, 7.474021911621094, 0.0)
        ],

        rightWaypoints: reflectWaypoints(
            makeWaypoint(4.4603095054626465, 7.474021911621094, 0.0)
        ),

        eventMarkers: [
            makeEventMarker("pivot raise", 0, 0)
        ]
    },

    neutralmiddle: {
        /*
         * Global waypoints 1 through 22 from SOTM.traj.
         * Because start contributes global waypoint 0, these are local 0..21.
         */
        leftWaypoints: [
            // 1
            makeWaypoint(4.4603095054626465, 7.474021911621094, 0.0),

            // 2
            makeWaypoint(4.809019565582275, 7.477536201477051, 0.0),

            // 3
            makeWaypoint(7.610898208618164, 7.091128826141357, -1.4711286226200226),

            // 4
            makeWaypoint(7.637355518341065, 5.014767646789551, -0.03591356551337211),

            // 5
            makeWaypoint(6.150235176086426, 5.5619797706604, 3.141592653589793),

            // 6 - SHOOT START
            makeWaypoint(3.0093276500701904, 5.499744415283203, 3.141592653589793),

            // 7
            makeWaypoint(2.9915871620178223, 5.483294486999512, -0.7301040444124813),

            // 8 - SHOOT STOP
            makeWaypoint(2.9354302883148193, 6.875729084014893, -1.17555532309072),

            // 9
            makeWaypoint(3.2754716873168945, 7.364711284637451, -0.020472520090271434),

            // 10 - first return to start area
            makeWaypoint(4.448259353637695, 7.463142395019531, 0.0),

            // 11
            makeWaypoint(4.809019565582275, 7.477536201477051, 0.0),

            // 12
            makeWaypoint(7.610898208618164, 7.091128826141357, -1.4711286226200226),

            // 13
            makeWaypoint(7.621798992156982, 5.009193420410156, -2.4227627089480697),

            // 14
            makeWaypoint(6.150235176086426, 5.5619797706604, 3.141592653589793),

            // 15 - SHOOT START
            makeWaypoint(3.006711959838867, 5.49761438369751, 3.141592653589793),

            // 16
            makeWaypoint(2.9915871620178223, 5.483294486999512, -0.7301040444124813),

            // 17 - SHOOT STOP
            makeWaypoint(2.9235498905181885, 6.875729084014893, -1.02652324394404),

            // 18
            makeWaypoint(3.279958963394165, 7.350941181182861, -0.020472520090271434),

            // 19 - second return
            makeWaypoint(4.456109046936035, 7.4697442054748535, 0.0),

            // 20
            makeWaypoint(4.824398517608643, 7.4697442054748535, 0.0),

            // 21
            makeWaypoint(7.610898208618164, 7.091128826141357, -1.4711286226200226),

            // 22
            makeWaypoint(7.610898208618164, 7.091128826141357, -1.4711286226200226)
        ],

        rightWaypoints: reflectWaypoints(
            makeWaypoint(4.4603095054626465, 7.474021911621094, 0.0),
            makeWaypoint(4.809019565582275, 7.477536201477051, 0.0),
            makeWaypoint(7.610898208618164, 7.091128826141357, -1.4711286226200226),
            makeWaypoint(7.637355518341065, 5.014767646789551, -0.03591356551337211),
            makeWaypoint(6.150235176086426, 5.5619797706604, 3.141592653589793),
            makeWaypoint(3.0093276500701904, 5.499744415283203, 3.141592653589793),
            makeWaypoint(2.9915871620178223, 5.483294486999512, -0.7301040444124813),
            makeWaypoint(2.9354302883148193, 6.875729084014893, -1.17555532309072),
            makeWaypoint(3.2754716873168945, 7.364711284637451, -0.020472520090271434),
            makeWaypoint(4.448259353637695, 7.463142395019531, 0.0),
            makeWaypoint(4.809019565582275, 7.477536201477051, 0.0),
            makeWaypoint(7.610898208618164, 7.091128826141357, -1.4711286226200226),
            makeWaypoint(7.621798992156982, 5.009193420410156, -2.4227627089480697),
            makeWaypoint(6.150235176086426, 5.5619797706604, 3.141592653589793),
            makeWaypoint(3.006711959838867, 5.49761438369751, 3.141592653589793),
            makeWaypoint(2.9915871620178223, 5.483294486999512, -0.7301040444124813),
            makeWaypoint(2.9235498905181885, 6.875729084014893, -1.02652324394404),
            makeWaypoint(3.279958963394165, 7.350941181182861, -0.020472520090271434),
            makeWaypoint(4.456109046936035, 7.4697442054748535, 0.0),
            makeWaypoint(4.824398517608643, 7.4697442054748535, 0.0),
            makeWaypoint(7.610898208618164, 7.091128826141357, -1.4711286226200226),
            makeWaypoint(7.610898208618164, 7.091128826141357, -1.4711286226200226)
        ),

        constraints: [
            // Keep the first bump/shooting path away from the protected areas.
            makeConstraint(5, 7, {
                type: "KeepOutCircle",
                props: {
                    x: toExpr(4.612211856842041, "m"),
                    y: toExpr(6.084839515686035, "m"),
                    r: toExpr(0.9316006363189093, "m")
                }
            }),

            makeConstraint(5, 7, {
                type: "KeepOutCircle",
                props: {
                    x: toExpr(4.612211856842041, "m"),
                    y: toExpr(1.978060484313966, "m"),
                    r: toExpr(0.9316006363189093, "m")
                }
            }),

            // Stop at the end of the first loop.
            makeConstraint(9, undefined, {
                type: "StopPoint",
                props: {}
            }),

            // Stop at the end of the second loop.
            makeConstraint(18, undefined, {
                type: "StopPoint",
                props: {}
            }),

            /*
             * SOTM WINDOW #1
             * Global 6 -> 8 = local 5 -> 7
             */
            makeConstraint(5, 7, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(1.5, "m/s")
                }
            }),

            /*
             * SOTM WINDOW #2
             * Global 15 -> 17 = local 14 -> 16
             */
            makeConstraint(14, 16, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(1.5, "m/s")
                }
            })

            /*
             * PointAt goes here once ConstraintDefinitions.ts is used to confirm
             * the exact local props shape:
             *
             * global 6 -> 8  => local 5 -> 7
             * global 15 ->17 => local 14 ->16
             *
             * Target:
             *     HUB_X = 4.63710355758667 m
             *     HUB_Y = 4.025846004486084 m
             */
        ],

        eventMarkers: [
            // Source SOTM.traj global target 1
            makeEventMarker("stop shooting", 0, 0.2),

            // global target 3
            makeEventMarker("feed in", 2, 0),

            // global target 5
            makeEventMarker("feed stop", 4, -0.3),

            // global target 6
            makeEventMarker("shoot", 5, -0.1),

            // global target 8
            makeEventMarker("stop shooting", 7, 0),

            // global target 14
            makeEventMarker("feed stop", 13, -0.3),

            // global target 15
            makeEventMarker("shoot", 14, -0.1),

            // global target 17
            makeEventMarker("stop shooting", 16, 0)
        ]
    },

    neutralbump: {
        /** @todo */
    },

    depot: {
        /** @todo */
    },

    human: {
        /** @todo */
    }
};
