import { makeWaypoint, reflectWaypoints, makeEventMarker, makeConstraint, toExpr, AllLocationProperties } from "../types";

export const allLocationProperties: AllLocationProperties = {
    start: {
        leftWaypoints: [
            makeWaypoint(4.4603095054626465, 7.474021911621094, 0.0)
        ],
        rightWaypoints: reflectWaypoints(
            makeWaypoint(4.4603095054626465, 7.474021911621094, 0.0)
        )
    },
    neutralmiddle: {
        leftWaypoints: [
            makeWaypoint(4.4603095054626465, 7.474021911621094, 0.0),
            makeWaypoint(4.809019565582275, 7.477536201477051, 0.0),
            makeWaypoint(7.610898208618164, 7.091128826141357, -1.4711286226200226),
            makeWaypoint(7.6373555183410645, 5.014767646789551, -2.0576957311828057),
            makeWaypoint(5.6979265213012695, 7.4296064376831055, 3.141592653589793),
            makeWaypoint(3.2754716873168945, 7.364711284637451, -1.1729889971693404, true, true, true)
        ],
        rightWaypoints: reflectWaypoints(
            makeWaypoint(4.4603095054626465, 7.474021911621094, 0.0),
            makeWaypoint(4.809019565582275, 7.477536201477051, 0.0),
            makeWaypoint(7.610898208618164, 7.091128826141357, -1.4711286226200226),
            makeWaypoint(7.6373555183410645, 5.014767646789551, -2.0576957311828057),
            makeWaypoint(5.6979265213012695, 7.4296064376831055, 3.141592653589793),
            makeWaypoint(3.2754716873168945, 7.364711284637451, -1.1729889971693404, true, true, true)
        ),
        constraints: [
            makeConstraint(5, undefined, { type: "StopPoint", props: {} }),
            makeConstraint(2, 3, { type: "MaxVelocity", props: {max: toExpr(1, "m/s")} })
        ],
        eventMarkers: [
            makeEventMarker("stop shooting", 0, 0),
            makeEventMarker("feed in", 2, 0),
            makeEventMarker("feed stop", 5, -0.3),
        ]
    },
    neutralbump: {
        leftWaypoints: [
            makeWaypoint(3.2754716873168945, 7.364711284637451, -1.1729889971693404),
            makeWaypoint(4.809019565582275, 7.477536201477051, 0.0),
            makeWaypoint(7.610898208618164, 7.091128826141357, -1.4711286226200226),
            makeWaypoint(7.679258060455322, 5.416146278381348, -2.0576957311828057),
            makeWaypoint(5.850483417510986, 4.746649742126465, 1.5495226816598229),
            makeWaypoint(5.693442344665527, 7.406031608581543, 3.131788918491988),
            makeWaypoint(2.770551919937134, 7.13850736618042, -1.0286093969667478, true, true, true)
        ],
        rightWaypoints: reflectWaypoints(
            makeWaypoint(3.2754716873168945, 7.364711284637451, -1.1729889971693404),
            makeWaypoint(4.809019565582275, 7.477536201477051, 0.0),
            makeWaypoint(7.610898208618164, 7.091128826141357, -1.4711286226200226),
            makeWaypoint(7.679258060455322, 5.416146278381348, -2.0576957311828057),
            makeWaypoint(5.850483417510986, 4.746649742126465, 1.5495226816598229),
            makeWaypoint(5.693442344665527, 7.406031608581543, 3.131788918491988),
            makeWaypoint(2.770551919937134, 7.13850736618042, -1.0286093969667478, true, true, true)
        ),
        constraints: [
            makeConstraint(6, undefined, { type: "StopPoint", props: {} }),
            makeConstraint(3, 4, { type: "MaxVelocity", props: {max: toExpr(1, "m/s")} })
        ],
        eventMarkers: [
            makeEventMarker("stop shooting", 0, 0),
            makeEventMarker("feed in", 1, 0),
            makeEventMarker("feed stop", 5, -0.3),
        ]
    },
    depot: {
        /** @todo */
    },
    human: {
        /** @todo */
    }
}