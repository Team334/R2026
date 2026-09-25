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

export const allLocationProperties: AllLocationProperties = {
    start: {
        leftWaypoints: [
            waypoint(
                4.437711238861084,
                0.43769314885139465,
                1.5707963267948966,
                29
            )
        ],

        rightWaypoints: reflectWaypoints(
            waypoint(
                4.437711238861084,
                0.43769314885139465,
                1.5707963267948966,
                29
            )
        ),

        eventMarkers: [
            makeEventMarker("pivot raise", 0, 0)
        ],

        constraints: [
            makeConstraint(0, 1, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(5.12, "m/s")
                }
            }),
            makeConstraint(0, 17, {
                type: "MaxAcceleration",
                props: {
                    max: toExpr(10.0, "m/s^2")
                }
            })
        ]
    },

    neutralmiddle: {
        leftWaypoints: [
            waypoint(4.437711238861084, 0.43769314885139465, 1.5707963267948966, 29),
            waypoint(8.128582000732422, 0.951529436492921, 1.6839471199122955, 32),
            waypoint(7.58929443359375, 3.22220986404419, 2.0863448821145987, 29),
            waypoint(5.7869415283203125, 2.552360468292237, 1.5469914909759674, 38),
            waypoint(3.0905120372772217, 2.129444532775878, 0.8934711829610875, 3),
            waypoint(3.104701280593872, 2.1407994560241708, 0.8904947688844378, 34, false, false),
            waypoint(2.891824960708618, 0.7642001441955574, 0.0, 22, true, false)
        ],

        rightWaypoints: reflectWaypoints(
            waypoint(4.437711238861084, 0.43769314885139465, 1.5707963267948966, 29),
            waypoint(8.128582000732422, 0.951529436492921, 1.6839471199122955, 32),
            waypoint(7.58929443359375, 3.22220986404419, 2.0863448821145987, 29),
            waypoint(5.7869415283203125, 2.552360468292237, 1.5469914909759674, 38),
            waypoint(3.0905120372772217, 2.129444532775878, 0.8934711829610875, 3),
            waypoint(3.104701280593872, 2.1407994560241708, 0.8904947688844378, 34, false, false),
            waypoint(2.891824960708618, 0.7642001441955574, 0.0, 22, true, false)
        ),

        constraints: [
            makeConstraint(0, 3, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(2.0, "m/s")
                }
            }),
            makeConstraint(3, 5, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(2.0, "m/s")
                }
            }),
            makeConstraint(5, 6, {
                type: "PointAt",
                props: {
                    x: toExpr(4.637411117553711, "m"),
                    y: toExpr(4.031760215759277, "m"),
                    tolerance: toExpr(5, "deg"),
                    flip: false
                }
            }),
            makeConstraint(4, undefined, {
                type: "StopPoint",
                props: {}
            }),
            makeConstraint(6, undefined, {
                type: "StopPoint",
                props: {}
            }),
            makeConstraint(5, 6, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(1.0, "m/s")
                }
            })
        ],

        eventMarkers: [
            makeEventMarker("stop shooting", 0, 0.2),
            makeEventMarker("feed in", 3, 0),
            makeEventMarker("shoot", 4, -0.1),
            makeEventMarker("stop shooting", 6, 0)
        ]
    },

    neutralbump: {
        leftWaypoints: [
            waypoint(2.891824960708618, 0.7642001441955574, 0.0, 22, true, false),
            waypoint(4.438725471496582, 0.43778985000610415, 1.5707963267948966, 16),
            waypoint(5.90047550201416, 0.693241053009034, 1.5707963267948966, 23),
            waypoint(6.0074944496154785, 1.976436071777344, 1.5707963267948966, 26, true, false),
            waypoint(6.0074944496154785, 3.600603037261963, 1.5413932999533333, 15, true, false),
            waypoint(6.310672283172607, 3.990403108978272, 0.0, 34),
            waypoint(7.106773853302002, 2.4955930046081547, -3.141592653589793, 26),
            waypoint(5.832298755645752, 2.4175557426452633, 1.5707963267948966, 38),
            waypoint(3.104701042175293, 2.1407994560241708, 0.0, 33, true, false),
            waypoint(2.920208215713501, 0.7642001441955574, 0.0, 21, true, false),
            waypoint(4.437710762023926, 0.4463900856018061, 1.5707963267948966, 25),
            waypoint(7.787977695465088, 0.5938991836547867, 1.5707963267948966, 40)
        ],

        rightWaypoints: reflectWaypoints(
            waypoint(2.891824960708618, 0.7642001441955574, 0.0, 22, true, false),
            waypoint(4.438725471496582, 0.43778985000610415, 1.5707963267948966, 16),
            waypoint(5.90047550201416, 0.693241053009034, 1.5707963267948966, 23),
            waypoint(6.0074944496154785, 1.976436071777344, 1.5707963267948966, 26, true, false),
            waypoint(6.0074944496154785, 3.600603037261963, 1.5413932999533333, 15, true, false),
            waypoint(6.310672283172607, 3.990403108978272, 0.0, 34),
            waypoint(7.106773853302002, 2.4955930046081547, -3.141592653589793, 26),
            waypoint(5.832298755645752, 2.4175557426452633, 1.5707963267948966, 38),
            waypoint(3.104701042175293, 2.1407994560241708, 0.0, 33, true, false),
            waypoint(2.920208215713501, 0.7642001441955574, 0.0, 21, true, false),
            waypoint(4.437710762023926, 0.4463900856018061, 1.5707963267948966, 25),
            waypoint(7.787977695465088, 0.5938991836547867, 1.5707963267948966, 40)
        ),

        constraints: [
            makeConstraint(0, 2, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(4.0, "m/s")
                }
            }),
            makeConstraint(2, 8, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(2.0, "m/s")
                }
            }),
            makeConstraint(5, 5, {
                type: "KeepOutCircle",
                props: {
                    x: toExpr(4.61231791973114, "m"),
                    y: toExpr(4.0446482971310616, "m"),
                    r: toExpr(0.6450773307498116, "m")
                }
            }),
            makeConstraint(8, undefined, {
                type: "StopPoint",
                props: {}
            }),
            makeConstraint(8, 9, {
                type: "MaxVelocity",
                props: {
                    max: toExpr(1.0, "m/s")
                }
            }),
            makeConstraint(8, 9, {
                type: "PointAt",
                props: {
                    x: toExpr(4.621539115905762, "m"),
                    y: toExpr(4.029186248779297, "m"),
                    tolerance: toExpr(5, "deg"),
                    flip: false
                }
            }),
            makeConstraint(9, undefined, {
                type: "StopPoint",
                props: {}
            })
        ],

        eventMarkers: [
            makeEventMarker("feed in", 2, 0),
            makeEventMarker("feed stop", 6, 0),
            makeEventMarker("shoot", 8, -0.1),
            makeEventMarker("stop shooting", 9, 0)
        ]
    },

    depot: {
        /** @todo */
    },

    human: {
        /** @todo */
    }
};