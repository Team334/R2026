import {
    makeWaypoint,
    reflectWaypoints,
    makeEventMarker,
    makeConstraint,
    toExpr,
    AllLocationProperties
} from "../types";

const neutralMiddlePathLeft = [
    makeWaypoint(
        4.424533843994141,
        7.610918521881104,
        -1.5707963267948966
    ),
    makeWaypoint(
        8.128582000732422,
        7.11137056350708,
        -1.6839471199122955
    ),
    makeWaypoint(
        7.750766754150391,
        4.440641403198242,
        -1.592532512087892
    ),
    makeWaypoint(
        5.7869415283203125,
        5.510539531707764,
        -1.5707963267948966
    ),
    makeWaypoint(
        3.127305746078491,
        5.631696701049805,
        -0.8934711829610875,
        true,
        false
    ),
    makeWaypoint(
        2.891824960708618,
        7.298699855804443,
        0.0,
        true,
        false
    )
];

// Swipe 2 as it appears in the manually-fixed Franco.traj.
// There is NO extra handoff waypoint here. The 6th waypoint of
// neutralmiddle is followed directly by this path when the locations
// are stitched by the generator.
const neutralBumpPathLeft = [
    makeWaypoint(
        4.438725471496582,
        7.625110149383545,
        -1.5707963267948966
    ),
    makeWaypoint(
        5.90047550201416,
        7.369658946990967,
        -1.5707963267948966
    ),
    makeWaypoint(
        6.0074944496154785,
        6.086463928222656,
        -1.5707963267948966,
        true,
        false
    ),
    makeWaypoint(
        5.931700229644775,
        4.635540962219238,
        -1.5413932999533333,
        true,
        false
    ),
    makeWaypoint(
        5.942527770996094,
        3.9858744144439697,
        0.0
    ),
    makeWaypoint(
        7.859044551849365,
        4.137463569641113,
        1.5469907766412327
    ),
    makeWaypoint(
        7.891528129577637,
        5.913218975067139,
        3.141592653589793
    ),
    makeWaypoint(
        5.79311990737915,
        5.488104820251465,
        -1.5707963267948966
    ),
    makeWaypoint(
        3.0953502655029297,
        5.622433662414551,
        0.0,
        true,
        false
    ),
    makeWaypoint(
        2.920208215713501,
        7.298699855804443,
        0.0,
        true,
        false
    ),
    makeWaypoint(
        4.437466621398926,
        7.613180160522461,
        -1.5707963267948966
    ),
    makeWaypoint(
        7.787977695465088,
        7.469000816345215,
        -1.5707963267948966
    )
];

export const allLocationProperties: AllLocationProperties = {
    start: {
        // leftWaypoints: [neutralMiddlePathLeft[0]],
        // rightWaypoints: reflectWaypoints(neutralMiddlePathLeft[0]),
        eventMarkers: [
            makeEventMarker("pivot raise", 0, 0),
            makeEventMarker("pivot lower", 0, 0.4),
            makeEventMarker("stop shooting", 0, 0)
        ],
        constraints: [
            makeConstraint(0, undefined, {
                type: "StopPoint",
                props: {}
            })
        ]
    },

    neutralmiddle: {
        leftWaypoints: neutralMiddlePathLeft,
        rightWaypoints: reflectWaypoints(...neutralMiddlePathLeft),
        constraints: [
            makeConstraint(0, 5, {
                type: "MaxAcceleration",
                props: {
                    max: toExpr(10.0, "m/s^2")
                }
            }),
            makeConstraint(1, 3, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(2.0, "m/s")
                }
            }),
            makeConstraint(4, 5, {
                type: "PointAt",
                props: {
                    x: toExpr(4.637411117553711, "m"),
                    y: toExpr(4.031760215759277, "m"),
                    tolerance: toExpr(1, "deg"),
                    flip: false
                }
            }),
            makeConstraint(0, 1, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(5.12, "m/s")
                }
            }),
            makeConstraint(3, 4, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(1.5, "m/s")
                }
            }),
            makeConstraint(4, 5, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(0.5, "m/s")
                }
            }),
            makeConstraint(4, undefined, {
                type: "StopPoint",
                props: {}
            })
        ],
        eventMarkers: [
            makeEventMarker("feed in", 1, -0.1),
            makeEventMarker("feed stop", 3, 0),
            makeEventMarker("shoot", 4, 0)
        ]
    },

    neutralbump: {
        leftWaypoints: neutralBumpPathLeft,
        rightWaypoints: reflectWaypoints(...neutralBumpPathLeft),
        constraints: [
            makeConstraint(0, 1, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(5.0, "m/s")
                }
            }),
            // Swipe 2 WP4 -> local WP4
            makeConstraint(4, 4, {
                type: "KeepOutCircle",
                props: {
                    x: toExpr(4.61231791973114, "m"),
                    y: toExpr(4.0446482971310616, "m"),
                    r: toExpr(0.6450773307498116, "m")
                }
            }),

            // Swipe 2 WP1 -> WP7
            makeConstraint(1, 7, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(2.0, "m/s")
                }
            }),

            // Swipe 2 WP8
            makeConstraint(8, undefined, {
                type: "StopPoint",
                props: {}
            }),

            // Swipe 2 WP8 -> WP9
            makeConstraint(8, 9, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(0.5, "m/s")
                }
            }),

            makeConstraint(8, 9, {
                type: "PointAt",
                props: {
                    x: toExpr(4.621539115905762, "m"),
                    y: toExpr(4.029186248779297, "m"),
                    tolerance: toExpr(1, "deg"),
                    flip: false
                }
            }),

            // Swipe 2 WP9
            makeConstraint(9, undefined, {
                type: "StopPoint",
                props: {}
            }),

            // Swipe 2 WP7 -> WP8
            makeConstraint(7, 8, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(1.5, "m/s")
                }
            })
        ],
        eventMarkers: [
            makeEventMarker("feed in", 1, 0),
            makeEventMarker("feed stop", 7, 0),
            makeEventMarker("shoot", 8, -0.1),
            makeEventMarker("stop shooting", 9, 0)
        ]
    },

    depot: {},
    human: {}
};
