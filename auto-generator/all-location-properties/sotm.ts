import {
    makeWaypoint,
    reflectWaypoints,
    makeEventMarker,
    makeConstraint,
    toExpr,
    AllLocationProperties
} from "../types";

function waypoint(
    x: number,
    y: number,
    heading: number,
    intervals: number,
    fixTranslation = true,
    fixHeading = true
) {
    const wp = makeWaypoint(
        x,
        y,
        heading,
        fixTranslation,
        fixHeading,
        false
    );

    wp.intervals = intervals;
    return wp;
}

// Swipe 1: SOTMBumpFirstSwipe
const neutralMiddlePathLeft = [
    waypoint(
        4.424533843994141,
        7.610918521881104,
        -1.5707963267948966,
        29
    ),
    waypoint(
        8.128582000732422,
        7.11137056350708,
        -1.6839471199122955,
        34
    ),
    waypoint(
        7.750766754150391,
        4.440641403198242,
        -1.592532512087892,
        28
    ),
    waypoint(
        5.7869415283203125,
        5.510539531707764,
        -1.5707963267948966,
        45
    ),
    waypoint(
        3.127305746078491,
        5.631696701049805,
        -0.8934711829610875,
        73,
        true,
        false
    ),
    waypoint(
        2.891824960708618,
        7.298699855804443,
        0.0,
        22,
        true,
        false
    )
];

// Swipe 2: SOTMBumpSecondSwipe
// Index 0 is the handoff point copied from neutralmiddle's final point.
const neutralBumpPathLeft = [
    ...neutralMiddlePathLeft.slice(-1),

    waypoint(
        4.438725471496582,
        7.625110149383545,
        -1.5707963267948966,
        16
    ),
    waypoint(
        5.90047550201416,
        7.369658946990967,
        -1.5707963267948966,
        23
    ),
    waypoint(
        6.0074944496154785,
        6.086463928222656,
        -1.5707963267948966,
        24,
        true,
        false
    ),
    waypoint(
        5.931700229644775,
        4.635540962219238,
        -1.5413932999533333,
        16,
        true,
        false
    ),
    waypoint(
        5.942527770996094,
        3.9858744144439697,
        0.0,
        33
    ),
    waypoint(
        7.859044551849365,
        4.137463569641113,
        1.5469907766412327,
        31
    ),
    waypoint(
        7.891528129577637,
        5.913218975067139,
        3.141592653589793,
        34
    ),
    waypoint(
        5.832298755645752,
        5.645344257354736,
        -1.5707963267948966,
        46
    ),
    waypoint(
        3.104701042175293,
        5.92210054397583,
        0.0,
        60,
        true,
        false
    ),
    waypoint(
        2.920208215713501,
        7.298699855804443,
        0.0,
        21,
        true,
        false
    ),
    waypoint(
        4.437466621398926,
        7.613180160522461,
        -1.5707963267948966,
        25
    ),
    waypoint(
        7.787977695465088,
        7.469000816345215,
        -1.5707963267948966,
        40
    )
];

const keepInField = {
    type: "KeepInRectangle" as const,
    props: {
        x: toExpr(0.0, "m"),
        y: toExpr(0.0, "m"),
        w: toExpr(16.541, "m"),
        h: toExpr(8.0692, "m")
    }
};

export const allLocationProperties: AllLocationProperties = {
    start: {
        leftWaypoints: [neutralMiddlePathLeft[0]],
        rightWaypoints: reflectWaypoints(neutralMiddlePathLeft[0]),
        eventMarkers: [
            makeEventMarker("pivot raise", 0, 0),
            makeEventMarker("pivot lower", 0, 0.4)
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
            // Stop only at the end of neutralmiddle.
            makeConstraint(5, undefined, {
                type: "StopPoint",
                props: {}
            }),

            makeConstraint(0, 5, keepInField),

            // Original Swipe 1: 1 -> 3
            makeConstraint(1, 3, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(2.0, "m/s")
                }
            }),

            // Original Swipe 1: 4 -> 5
            makeConstraint(4, 5, {
                type: "PointAt",
                props: {
                    x: toExpr(4.637411117553711, "m"),
                    y: toExpr(4.031760215759277, "m"),
                    tolerance: toExpr(1, "deg"),
                    flip: false
                }
            }),

            // Original Swipe 1: 0 -> 1
            makeConstraint(0, 1, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(5.12, "m/s")
                }
            }),

            // Original Swipe 1: 3 -> 4
            makeConstraint(3, 4, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(1.5, "m/s")
                }
            }),

            // Original Swipe 1: 4 -> 5
            makeConstraint(4, 5, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(0.5, "m/s")
                }
            }),

            // Original Swipe 1: 0 -> 5
            makeConstraint(0, 5, {
                type: "MaxAcceleration",
                props: {
                    max: toExpr(10.0, "m/s^2")
                }
            })
        ],
        eventMarkers: [
            makeEventMarker("feed in", 1, -0.1),
            makeEventMarker("feed stop", 3, 0),
            makeEventMarker("shoot", 4, 0),
            makeEventMarker("stop shooting", 5, 0)
        ]
    },

    neutralbump: {
        leftWaypoints: neutralBumpPathLeft,
        rightWaypoints: reflectWaypoints(...neutralBumpPathLeft),
        constraints: [
            // Final point = Swipe 2 WP11, now index 12.
            makeConstraint(12, undefined, {
                type: "StopPoint",
                props: {}
            }),

            // Whole stitched neutralbump path.
            makeConstraint(0, 12, keepInField),

            // Original Swipe 2 WP4 -> stitched WP5.
            makeConstraint(5, 5, {
                type: "KeepOutCircle",
                props: {
                    x: toExpr(4.61231791973114, "m"),
                    y: toExpr(4.0446482971310616, "m"),
                    r: toExpr(0.6450773307498116, "m")
                }
            }),

            // Original Swipe 2 WP1 -> WP7 becomes WP2 -> WP8.
            makeConstraint(2, 8, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(2.0, "m/s")
                }
            }),

            // Original Swipe 2 WP8 becomes stitched WP9.
            makeConstraint(9, undefined, {
                type: "StopPoint",
                props: {}
            }),

            // Original Swipe 2 WP8 -> WP9 becomes WP9 -> WP10.
            makeConstraint(9, 10, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(0.5, "m/s")
                }
            }),

            // Original Swipe 2 WP8 -> WP9 becomes WP9 -> WP10.
            makeConstraint(9, 10, {
                type: "PointAt",
                props: {
                    x: toExpr(4.621539115905762, "m"),
                    y: toExpr(4.029186248779297, "m"),
                    tolerance: toExpr(1, "deg"),
                    flip: false
                }
            }),

            // Original Swipe 2 WP9 becomes stitched WP10.
            makeConstraint(10, undefined, {
                type: "StopPoint",
                props: {}
            }),

            // Original Swipe 2 WP0 -> WP11 becomes WP1 -> WP12.
            makeConstraint(1, 12, {
                type: "MaxAcceleration",
                props: {
                    max: toExpr(10.0, "m/s^2")
                }
            }),

            // Original Swipe 2 WP0 -> WP1 becomes WP1 -> WP2.
            makeConstraint(1, 2, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(5.0, "m/s")
                }
            }),

            // Original Swipe 2 WP7 -> WP8 becomes WP8 -> WP9.
            makeConstraint(8, 9, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(1.5, "m/s")
                }
            })
        ],
        eventMarkers: []
    },

    depot: {},
    human: {}
};