import { makeWaypoint, reflectWaypoints, makeEventMarker, makeConstraint, toExpr, AllLocationProperties } from "../types";

export const allLocationProperties: AllLocationProperties = {
    start: {
        leftWaypoints: [
            makeWaypoint(3.5480897426605225, 7.474021911621094, 0.0)
        ],
        rightWaypoints: reflectWaypoints(
            makeWaypoint(3.5480897426605225, 7.474021911621094, 0.0)
        ),
        eventMarkers: [
            makeEventMarker("pivot lower", 0, 0.3)
        ]
    },
    neutralbump: {
        leftWaypoints: [
            makeWaypoint(5.576541423797607, 7.480874061584473, 0, true, true),
            makeWaypoint(5.955106735229492, 5.9437713623046875, -1.5707963267948966),
            makeWaypoint(5.955106735229492, 4.313876152038574, -1.5707963267948966),
            makeWaypoint(5.693442344665527, 7.406031608581543, 3.131788918491988),
            makeWaypoint(2.770551919937134, 7.13850736618042, -1.0286093969667478, true, true, true)
        ],
        rightWaypoints: reflectWaypoints(
            makeWaypoint(5.576541423797607, 7.480874061584473, 0, true, true),
            makeWaypoint(5.955106735229492, 5.9437713623046875, -1.5707963267948966),
            makeWaypoint(5.955106735229492, 4.313876152038574, -1.5707963267948966),
            makeWaypoint(5.693442344665527, 7.406031608581543, 3.131788918491988),
            makeWaypoint(2.770551919937134, 7.13850736618042, -1.0286093969667478, true, true, true)
        ),
        constraints: [
            makeConstraint(5, undefined, {type: "StopPoint", props: {}}),
            makeConstraint(1, 2, {type: "KeepInLane", props: {tolerance: toExpr(0.03, "m")}})
        ],
        eventMarkers: [
            makeEventMarker("stop shooting", 0, 0),
            makeEventMarker("feed in", 1, 0),
            makeEventMarker("feed stop", 2, 0.4),
        ]
    },
    neutralmiddle: {
        leftWaypoints: [
            makeWaypoint(3.7149038314819336, 7.477536201477051, 0.0, true, false),
            makeWaypoint(4.809019565582275, 7.520789623260498, 0.0),
            makeWaypoint(7.484222412109375, 6.948218822479248, -1.4711286226200226),
            makeWaypoint(7.4373555183410645, 5.014767646789551, -2.0576957311828057),
            makeWaypoint(5.6979265213012695, 7.4296064376831055, 3.141592653589793),
            makeWaypoint(3.2754716873168945, 7.364711284637451, -1.1729889971693404, true, true, true)
        ],
        rightWaypoints: reflectWaypoints(
            makeWaypoint(3.7149038314819336, 7.477536201477051, 0.0, true, false),
            makeWaypoint(4.809019565582275, 7.520789623260498, 0.0),
            makeWaypoint(7.484222412109375, 6.948218822479248, -1.4711286226200226),
            makeWaypoint(7.4373555183410645, 5.014767646789551, -2.0576957311828057),
            makeWaypoint(5.6979265213012695, 7.4296064376831055, 3.141592653589793),
            makeWaypoint(3.2754716873168945, 7.364711284637451, -1.1729889971693404, true, true, true)
        ),
        constraints: [
            makeConstraint(5, undefined, {type: "StopPoint", props: {}})
        ],
        eventMarkers: [
            makeEventMarker("stop shooting", 0, 0),
            makeEventMarker("feed in", 2, 0),
            makeEventMarker("feed stop", 3, 0.4),
        ]
    },
    depot: {
        leftWaypoints: [
            makeWaypoint(1.1794016361236572, 7.166106700897217, -1.5707963267948966, true, false),
            makeWaypoint(0.4468134641647339, 7.149000644683838, -1.5707963267948966),
            makeWaypoint(0.4468134641647339, 5.474844455718994, -1.5707963267948966, true, false),
            makeWaypoint(0.6906939148902893, 5.215888500213623, 0, true, false),
            makeWaypoint(0.99637770652771, 5.633697986602783, 0.0, true, true, true)
        ],
        rightWaypoints: [
            makeWaypoint(1.5006606578826904, 5.137444972991943, 0.0, true, false),
            makeWaypoint(0.43300509452819824, 4.9178547859191895, 1.5708),
            makeWaypoint(0.44392159581184387, 6.838572978973389, 1.5708, true, false),
            makeWaypoint(0.7331758141517639, 6.146159648895264, 1.5708, true, false),
            makeWaypoint(0.99637770652771, 5.633697986602783, -1.5707963267948966, true, true, true)
        ],
        constraints: [
            makeConstraint(1, 2, {type: "KeepInLane", props: {tolerance: toExpr(0.03, "m")}}),
            makeConstraint(1, 2, {type: "MaxAngularVelocity", props: {max: toExpr(0, "rad")}}),
            makeConstraint(4, undefined, {type: "StopPoint", props: {}})
        ],
        eventMarkers: [
            makeEventMarker("stop shooting", 0, 0),
            makeEventMarker("feed in", 1, 0),
            makeEventMarker("feed stop", 3, 0.4),
        ]
    },
    climb: {
        leftWaypoints: [
            makeWaypoint(2.7496304512023926, 4.047032356262207, 0, true, false),
            makeWaypoint(0.8680239319801331, 2.0315675735473633, 3.141592653589793),
            makeWaypoint(0.8600184917449951, 2.784078598022461, 0, true, false)
        ],
        rightWaypoints: [
            // (same as left)
            makeWaypoint(2.7496304512023926, 4.047032356262207, 0, true, false),
            makeWaypoint(0.8680239319801331, 2.0315675735473633, 3.141592653589793),
            makeWaypoint(0.8600184917449951, 2.784078598022461, 0, true, false)
        ],
        constraints: [
            makeConstraint(0, 1, {type: "KeepOutCircle", props: {x: toExpr(0.91920355707407, "m"), y: toExpr(3.742240246385336, "m"), r: toExpr(0.8, "m")}}),
            makeConstraint(1, 2, {type: "KeepInLane", props: {tolerance: toExpr(0.03, "m")}}),
            makeConstraint(1, 2, {type: "MaxAngularVelocity", props: {max: toExpr(0, "rad/s")}}),
            makeConstraint(1, 2, {type: "MaxVelocity", props: {max: toExpr(1, "m/s")}})
        ],
        eventMarkers: [
            makeEventMarker("stop shooting", 0, 0),
            makeEventMarker("pivot raise", 0, 0.1),
            makeEventMarker("extend", 0, 0.3),
            makeEventMarker("climb", 2, 0)
        ]
    }
}