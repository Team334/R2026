/** @todo */
// import { AllLocationProperties, makeConstraint, makeEventMarker, makeWaypoint, reflectWaypoints, toExpr } from "../types";

// export const allLocationProperties: AllLocationProperties = {
//     start: {
//         leftWaypoints: [
//             makeWaypoint(3.419274091720581, 7.474021911621094, 0.0)
//         ],
//         centerWaypoints: [
//             makeWaypoint(3.5993714332580566, 4.025567054748535, 0.0)
//         ],
//         rightWaypoints: reflectWaypoints(
//             makeWaypoint(3.419274091720581, 7.474021911621094, 0.0)
//         ),
//         eventMarkers: [
//             makeEventMarker("pivot lower", 0, 0.3)
//         ]
//     },
//     neutralbump: {
//         leftWaypoints: [
//             makeWaypoint(2.9307680130004883, 7.31110954284668, 0, true, false),
//             makeWaypoint(5.697521209716797, 7.402477741241455, -0.006293),
//             makeWaypoint(5.955106735229492, 5.9437713623046875, -1.5707963267948966),
//             makeWaypoint(5.955106735229492, 4.313876152038574, -1.5707963267948966),
//             makeWaypoint(5.693442344665527, 7.406031608581543, 3.131788918491988),
//             makeWaypoint(2.770551919937134, 7.13850736618042, -1.0286093969667478, true, true, true)
//         ],
//         rightWaypoints: reflectWaypoints(
//             makeWaypoint(2.9307680130004883, 7.31110954284668, 0, true, false),
//             makeWaypoint(5.697521209716797, 7.402477741241455, -0.006293),
//             makeWaypoint(5.955106735229492, 5.9437713623046875, -1.5707963267948966),
//             makeWaypoint(5.955106735229492, 4.313876152038574, -1.5707963267948966),
//             makeWaypoint(5.693442344665527, 7.406031608581543, 3.131788918491988),
//             makeWaypoint(2.770551919937134, 7.13850736618042, -1.0286093969667478, true, true, true)
//         ),
//         constraints: [
//             makeConstraint(5, undefined, { type: "StopPoint", props: {} }),
//         ],
//         eventMarkers: [
//             makeEventMarker("stop shooting", 0, 0),
//             makeEventMarker("feed in", 1, 0),
//             makeEventMarker("feed stop", 4, 0.4),
//         ]
//     },
//     neutralmiddle: {
//         leftWaypoints: [
//             makeWaypoint(3.7149038314819336, 7.477536201477051, 0.0, true, false),
//             makeWaypoint(4.809019565582275, 7.465332508087158, 0.0),
//             makeWaypoint(7.484222412109375, 6.948218822479248, -1.4711286226200226),
//             makeWaypoint(7.468718528747559, 4.425135612487793, -2.0576957311828057),
//             makeWaypoint(5.6979265213012695, 7.4296064376831055, 3.141592653589793),
//             makeWaypoint(3.2754716873168945, 7.364711284637451, 3.141592653589793, true, false)
//         ],
//         rightWaypoints: reflectWaypoints(
//             makeWaypoint(3.7149038314819336, 7.477536201477051, 0.0, true, false),
//             makeWaypoint(4.809019565582275, 7.465332508087158, 0.0),
//             makeWaypoint(7.484222412109375, 6.948218822479248, -1.4711286226200226),
//             makeWaypoint(7.468718528747559, 4.425135612487793, -2.0576957311828057),
//             makeWaypoint(5.6979265213012695, 7.4296064376831055, 3.141592653589793),
//             makeWaypoint(3.2754716873168945, 7.364711284637451, 3.141592653589793, true, false)
//         ),
//         constraints: [
//             makeConstraint(5, -1, { type: "PointAt", props: { x: toExpr(4.624067783355713, "m"), y: toExpr(4.038748741149902, "m"), tolerance: toExpr(0.017, "rad"), flip: false } }),
//             makeConstraint(5, -1, { type: "MaxVelocity", props: { max: toExpr(2, "m/s") } })
//         ],
//         eventMarkers: [
//             makeEventMarker("stop shooting", 0, 0),
//             makeEventMarker("feed in", 2, 0),
//             makeEventMarker("feed stop", 3, 0.4),
//             makeEventMarker("shoot", 5, 0)
//         ]
//     },
//     depot: {
//         leftWaypoints: [
//             makeWaypoint(1.1794016361236572, 7.166106700897217, -1.5707963267948966, true, false),
//             makeWaypoint(0.4468134641647339, 7.149000644683838, -1.5707963267948966),
//             makeWaypoint(0.4468134641647339, 5.474844455718994, -1.5707963267948966, true, false),
//             makeWaypoint(0.6906939148902893, 5.215888500213623, 0, true, false),
//             makeWaypoint(0.99637770652771, 5.633697986602783, 0.0, true, false)
//         ],
//         centerWaypoints: [
//             makeWaypoint(0.9916285276412964, 5.015804767608643, 0.0, true, false),
//             makeWaypoint(0.4727896749973297, 5.050980091094971, 1.5708),
//             makeWaypoint(0.45200252532958984, 7.020524978637695, 0.0, true, false),

//         ],
//         rightWaypoints: [
//             makeWaypoint(1.5006606578826904, 5.137444972991943, 0.0, true, false),
//             makeWaypoint(0.43300509452819824, 4.9178547859191895, 1.5708),
//             makeWaypoint(0.44392159581184387, 6.838572978973389, 1.5708, true, false),
//             makeWaypoint(0.7331758141517639, 6.146159648895264, 1.5708, true, false),
//             makeWaypoint(0.99637770652771, 5.633697986602783, -1.5707963267948966, true, false)
//         ],
//         constraints: [
//             makeConstraint(1, 2, { type: "KeepInLane", props: { tolerance: toExpr(0.03, "m") } }),
//             makeConstraint(1, 2, { type: "MaxAngularVelocity", props: { max: toExpr(0, "rad") } }),
//             makeConstraint(4, -1, { type: "PointAt", props: { x: toExpr(4.624067783355713, "m"), y: toExpr(4.038748741149902, "m"), tolerance: toExpr(0.071, "rad"), flip: false } }),
//             makeConstraint(4, -1, { type: "MaxVelocity", props: { max: toExpr(2, "m/s") } })
//         ],
//         eventMarkers: [
//             makeEventMarker("stop shooting", 0, 0),
//             makeEventMarker("feed in", 1, 0),
//             makeEventMarker("feed stop", 3, 0.4),
//             makeEventMarker("shoot", 3, 0)
//         ]
//     }
// }