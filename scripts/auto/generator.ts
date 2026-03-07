import { Expr, Constraint, Trajectory, TRAJ_SCHEMA_VERSION, Waypoint, EventMarker, EventMarkerData, WaypointIDX } from "./choreo/DocumentTypes";

// @ts-ignore
import * as fs from "fs";

// @ts-ignore
import { exec } from "child_process";

// @ts-ignore
import os from "os";
import { ConstraintData } from "./choreo/ConstraintDefinitions";

// @ts-ignore
import readline from "readline/promises";

const CHOREO_CLI = os.homedir().replace(/\\/g, "/") + '/AppData/Local/choreo/choreo-cli';

// (assume running generator from R2026 dir)
const DEPLOY_DIR: string = './src/main/deploy';
const TRAJ_DIR: string = DEPLOY_DIR + '/choreo/';

const LAYOUT_DIR: string = DEPLOY_DIR + '/layouts';
const CONFIG: string = TRAJ_DIR + '/config.chor';

const FIELD_WIDTH = 8.0629;

const rl = readline.createInterface({
    // @ts-ignore
    input: process.stdin,

    // @ts-ignore
    output: process.stdout
});

async function prompt(q: string) {
  return await rl.question(q);
}

function toExpr(val: number, unit: string) : Expr {
  return { exp: `${val} ${unit}`, val: val };
}

/**
 * Load layout file.
 * 
 * @param name Name of layout without extension.
 */
function loadLayout(name: string) : Layout {
    const layoutData = fs.readFileSync(LAYOUT_DIR + '/' + name + '.json', 'utf-8');
    
    return JSON.parse(layoutData) as Layout;
}

function shiftParams(constraints: Constraint[], eventMarkers: EventMarker[], waypointOffset: number) : { constraints: Constraint[], eventMarkers: EventMarker[] } {
    const shiftedConstraints: Constraint[] = [];
    const shiftedEventMarkers: EventMarker[] = [];

    for (const constraint of constraints) {
        const shiftedConstraint: Constraint = structuredClone(constraint);

        if (!(typeof constraint.from === 'number' && typeof constraint.to === 'number')) { continue; }

        shiftedConstraint.from = (constraint.from as number) + waypointOffset;

        if (shiftedConstraint.to == -1) {
            // connect constraint to the next waypoint
            shiftedConstraint.to = shiftedConstraint.from + 1;
        } else {
            shiftedConstraint.to = (constraint.to as number) + waypointOffset
        }

        shiftedConstraints.push(shiftedConstraint);
    }

    for (const eventMarker of eventMarkers) {
        const shiftedEventMarker: EventMarker = structuredClone(eventMarker);

        if (!(typeof eventMarker.from.target === 'number')) { continue; }

        shiftedEventMarker.from.target = (eventMarker.from.target as number) + waypointOffset;

        shiftedEventMarkers.push(shiftedEventMarker);
    }

    return { constraints: shiftedConstraints, eventMarkers: shiftedEventMarkers };
}

function buildTrajectory(baseTraj: Trajectory, name: string, layout: Location[]) : Trajectory {
    let trajWaypoints: Waypoint<Expr>[] = [];
    let trajConstraints: Constraint[] = [];
    let trajEventMarkers: EventMarker[] = [];

    let waypointOffset: number = 0;

    for (const location of layout) {
        const side = location.split('_')[0] as Side;
        const params = locationParams[location.split('_')[1] as GenericLocation];

        const shifted = shiftParams(
            params.constraints ?? [], 
            params.eventMarkers ?? [], 
            waypointOffset
        );

        switch (side) {
            case "l":
                trajWaypoints.push(...(params.leftWaypoints ?? []));
                trajConstraints.push(...shifted.constraints);
                trajEventMarkers.push(...shifted.eventMarkers);

                waypointOffset += params.leftWaypoints?.length ?? 0;
                break;
    
            case "c":
                trajWaypoints.push(...(params.centerWaypoints ?? []));
                trajConstraints.push(...shifted.constraints);
                trajEventMarkers.push(...shifted.eventMarkers);

                waypointOffset += params.centerWaypoints?.length ?? 0;
                break;

            case "r":
                trajWaypoints.push(...(params.rightWaypoints ?? []));
                trajConstraints.push(...shifted.constraints);
                trajEventMarkers.push(...shifted.eventMarkers);

                waypointOffset += params.rightWaypoints?.length ?? 0;
                break;

            default:
                break;
        }
    }

    const updatedTraj: Trajectory = baseTraj;

    updatedTraj.name = name;

    updatedTraj.params.waypoints = trajWaypoints;
    updatedTraj.params.constraints.push(...trajConstraints);
    updatedTraj.events.push(...trajEventMarkers);

    return updatedTraj;
}


/**
 * Save trajectory to deploy/choreo.
 */
function saveTrajectory(traj: Trajectory) : void {
    fs.writeFileSync(TRAJ_DIR + traj.name + ".traj", JSON.stringify(traj, null, 2));
}

/**
 * Generate the trajectory through choreo cli. The trajectory must exist in deploy/choreo first.
 */
function generateTrajectory(traj: Trajectory) : void {
    exec(CHOREO_CLI + " --chor " + CONFIG + " --trajectory " + traj.name + ".traj" + " -g", (error: Error | null, stdout: string, stderr: string) => {
        if (error) {
            console.error("Choreo Error:", error);
            return;
        }

        console.log("Choreo Output:");
        console.log(stdout);
    });
}

type Side = "l" | "c" | "r";
type GenericLocation = "start" | "neutralbump" | "neutralmiddle" | "trench" | "depot" | "climb";

type Location = `${Side}_${GenericLocation}`;

type EventName = "pivot out" | "pivot in" | "feed in" | "feed stop" | "shoot" | "stop shooting" | "extend" | "climb";

interface LocationParams {
    leftWaypoints?: Waypoint<Expr>[],
    centerWaypoints?: Waypoint<Expr>[],
    rightWaypoints?: Waypoint<Expr>[],
    constraints?: Constraint[],
    eventMarkers?: EventMarker[]
}

interface Layout {
    name: string,
    layout: Location[]
}

const reflectWaypoints = (...waypoints: Waypoint<Expr>[]) => {
    return waypoints.map(waypoint => ({
        ...waypoint,
        y: toExpr(FIELD_WIDTH - waypoint.y.val, "m"),
        heading: toExpr(waypoint.heading.val, "rad")
    }));
};

const makeWaypoint = (x: number, y: number, heading: number, fixTranslation: boolean = true, fixHeading: boolean = true) : Waypoint<Expr> => {
    return {
        x: toExpr(x, "m"),
        y: toExpr(y, "m"),
        heading: toExpr(heading, "rad"),
        intervals: 0,
        split: false,
        fixTranslation: fixTranslation,
        fixHeading: fixHeading,
        overrideIntervals: false
    };
};

const makeConstraint = (from: number, to: number, data: ConstraintData) : Constraint => {
    return {
        from: from,
        to: to,
        data: data,
        enabled: true
    };
}

const makeEventMarker = (name: EventName, target: number, offset: number) : EventMarker => {
    return {
        name: name,
        from: {
            target: target, 
            offset: toExpr(offset, "s"), 
            targetTimestamp: undefined
        },
        event: null
    }
}

// SOTM
const locationParams: Record<GenericLocation, LocationParams> = {
    start: {
        leftWaypoints: [
            makeWaypoint(3.419274091720581, 7.474021911621094, 0.0)
        ],
        rightWaypoints: reflectWaypoints(
            makeWaypoint(3.419274091720581, 7.474021911621094, 0.0)
        )
    },
    neutralbump: {
        leftWaypoints: [
            makeWaypoint(2.9307680130004883, 7.31110954284668, 0, true, false),
            makeWaypoint(5.697521209716797, 7.402477741241455, -0.006293),
            makeWaypoint(5.955106735229492, 5.9437713623046875, -1.5707963267948966),
            makeWaypoint(5.971709251403809, 4.325946807861328, -1.6095124425448013),
            makeWaypoint(5.905695915222168, 7.3172287940979, 3.131788918491988),
            makeWaypoint(2.770551919937134, 7.13850736618042, 0, true, false)
        ],
        rightWaypoints: reflectWaypoints(
            makeWaypoint(2.9307680130004883, 7.31110954284668, 0, true, false),
            makeWaypoint(5.697521209716797, 7.402477741241455, -0.006293),
            makeWaypoint(5.955106735229492, 5.9437713623046875, -1.5707963267948966),
            makeWaypoint(5.971709251403809, 4.325946807861328, -1.6095124425448013),
            makeWaypoint(5.905695915222168, 7.3172287940979, 3.131788918491988),
            makeWaypoint(2.770551919937134, 7.13850736618042, 0, true, false)
        ),
        constraints: [
            makeConstraint(5, -1, {type: "PointAt", props: {x: toExpr(4.624067783355713, "m"), y: toExpr(4.038748741149902, "m"), tolerance: toExpr(0.017, "rad"), flip: false}}),
            makeConstraint(5, -1, {type: "MaxVelocity", props: {max: toExpr(2, "m/s")}})
        ],
        eventMarkers: [
            makeEventMarker("stop shooting", 0, 0),
            makeEventMarker("pivot out", 0, 0),
            makeEventMarker("feed in", 1, 0),
            makeEventMarker("feed stop", 4, 0.4),
            makeEventMarker("shoot", 5, 0)
        ]
    },
    neutralmiddle: {
        leftWaypoints: [
            makeWaypoint(3.7149038314819336, 7.477536201477051, 0.0, true, false),
            makeWaypoint(4.809019565582275, 7.465332508087158, 0.0),
            makeWaypoint(7.484222412109375, 6.948218822479248, -1.4711286226200226),
            makeWaypoint(7.468718528747559, 4.425135612487793, -2.0576957311828057),
            makeWaypoint(5.6979265213012695, 7.4296064376831055, 3.141592653589793),
            makeWaypoint(3.2754716873168945, 7.364711284637451, 3.141592653589793, true, false)
        ],
        rightWaypoints: reflectWaypoints(
            makeWaypoint(3.7149038314819336, 7.477536201477051, 0.0, true, false),
            makeWaypoint(4.809019565582275, 7.465332508087158, 0.0),
            makeWaypoint(7.484222412109375, 6.948218822479248, -1.4711286226200226),
            makeWaypoint(7.468718528747559, 4.425135612487793, -2.0576957311828057),
            makeWaypoint(5.6979265213012695, 7.4296064376831055, 3.141592653589793),
            makeWaypoint(3.2754716873168945, 7.364711284637451, 3.141592653589793, true, false)
        ),
        constraints: [
            makeConstraint(5, -1, {type: "PointAt", props: {x: toExpr(4.624067783355713, "m"), y: toExpr(4.038748741149902, "m"), tolerance: toExpr(0.017, "rad"), flip: false}}),
            makeConstraint(5, -1, {type: "MaxVelocity", props: {max: toExpr(2, "m/s")}})
        ],
        eventMarkers: [
            makeEventMarker("stop shooting", 0, 0),
            makeEventMarker("pivot out", 1, 0),
            makeEventMarker("feed in", 2, 0),
            makeEventMarker("feed stop", 3, 0.4),
            makeEventMarker("shoot", 5, 0)
        ]
    },
    trench: {
        leftWaypoints: [
            makeWaypoint(2.7739968299865723, 6.4496846199035645, 0, true, false)
        ],
        rightWaypoints: reflectWaypoints( 
            makeWaypoint(2.7739968299865723, 6.4496846199035645, 0, true, false)
        ),
        constraints: [
            makeConstraint(0, -1, {type: "MaxVelocity", props: {max: toExpr(2, "m/s")}})
        ]
    },
    depot: {
        leftWaypoints: [
            makeWaypoint(1.206515908241272, 6.921103477478027, -1.5707963267948966, true, false),
            makeWaypoint(0.4468134641647339, 7.149000644683838, -1.5707963267948966),
            makeWaypoint(0.4468134641647339, 5.474844455718994, -1.5707963267948966),
            makeWaypoint(1.004274845123291, 5.474844455718994, 0.0, true, false)
        ],
        rightWaypoints: [
            makeWaypoint(1.5006606578826904, 5.137444972991943, 0.0, true, false),
            makeWaypoint(0.43300509452819824, 4.9178547859191895, 1.5708),
            makeWaypoint(0.44392159581184387, 6.838572978973389, 1.5708),
            makeWaypoint(1.206515908241272, 6.921103477478027, -1.5707963267948966, true, false)
        ],
        constraints: [
            makeConstraint(1, 2, {type: "KeepInLane", props: {tolerance: toExpr(0.03, "m")}}),
            makeConstraint(3, -1, {type: "PointAt", props: {x: toExpr(4.624067783355713, "m"), y: toExpr(4.038748741149902, "m"), tolerance: toExpr(0.071, "rad"), flip: false}}),
            makeConstraint(3, -1, {type: "MaxVelocity", props: {max: toExpr(2, "m/s")}})
        ],
        eventMarkers: [
            makeEventMarker("stop shooting", 0, 0),
            makeEventMarker("pivot out", 0, 0),
            makeEventMarker("feed in", 1, 0),
            makeEventMarker("feed stop", 3, 0.4),
            makeEventMarker("shoot", 3, 0)
        ]
    },
    climb: {
        leftWaypoints: [
            makeWaypoint(2.7496304512023926, 4.047032356262207, 0, true, false),
            makeWaypoint(2.4865808486938477, 2.7771108150482178, 3.141592653589793),
            makeWaypoint(0.8038443922996521, 2.7895240783691406, 0, true, false)
        ],
        rightWaypoints: [
            // (same as left)
            makeWaypoint(2.7496304512023926, 4.047032356262207, 0, true, false),
            makeWaypoint(2.4865808486938477, 2.7771108150482178, 3.141592653589793),
            makeWaypoint(0.8038443922996521, 2.7895240783691406, 0, true, false)
        ],
        constraints: [
            makeConstraint(1, 2, {type: "KeepInLane", props: {tolerance: toExpr(0.03, "m")}}),
            makeConstraint(1, 2, {type: "MaxAngularVelocity", props: {max: toExpr(0, "rad/s")}}),
            makeConstraint(1, 2, {type: "MaxVelocity", props: {max: toExpr(1, "m/s")}})
        ],
        eventMarkers: [
            makeEventMarker("extend", 1, -0.5),
            makeEventMarker("climb", 2, 0)
        ]
    }
}

// SNM
// const locationParams: Record<GenericLocation, LocationParams> = {
//     start: {
//         leftWaypoints: [
//             makeWaypoint(3.4212963581085205, 5.941174030303955, -1.1441687766816608),
//             makeWaypoint(3.283968210220337, 7.041266441345215, 0)
//         ],
//         centerWaypoints: [makeWaypoint(3, 4, 0)]
//     },
//     neutralbump: {},
//     neutralmiddle: {
//         leftWaypoints: [
//             makeWaypoint(4.059676647186279, 7.521993637084961, -1.5599661588553948),
//             makeWaypoint(7.636203765869141, 6.131853103637695, -1.4711286226200226),
//             makeWaypoint(7.501785278320312, 4.563638210296631, -2.0576957311828057),
//             makeWaypoint(4.586862087249756, 7.546622276306152, -1.5708)
//         ],
//         eventMarkers: [
//             makeEventMarker("shoot", 3, 0)
//         ]
//     },
//     trench: {},
//     depot: {
//         leftWaypoints: [
//             makeWaypoint(0.4917987287044525, 6.870540618896484, -1.5707963267948966),
//             makeWaypoint(0.4917987287044525, 5.075254440307617, -1.5707963267948966)
//         ],
//         constraints: [
//             makeConstraint(0, 1, {type: "KeepInLane", props: {tolerance: toExpr(0.03, "m")}})
//         ],
//         eventMarkers: [
//             makeEventMarker("stop shooting", 0, 0)
//         ]
//     },
//     human: {},
//     climb: {}
// }

var baseTraj: Trajectory = {
    name: "",
    version: TRAJ_SCHEMA_VERSION,
    params: {
        waypoints: [],
        constraints: [
            {from: "first", data: {type: "StopPoint", props: {}}, enabled: true},
            {from: "last", data: {type: "StopPoint", props: {}}, enabled: true},
            {from: "first", to: "last", data: {type: "KeepOutCircle", props: {x: toExpr(4.625668669119477, "m"), y: toExpr(6.208901214599609, "m"), r: toExpr(0.826396949005302, "m")}}, enabled: true},
            {from: "first", to: "last", data: {type: "KeepOutCircle", props: {x: toExpr(4.625668669119477, "m"), y: toExpr(1.8539987854, "m"), r: toExpr(0.826396949005302, "m")}}, enabled: true},
            {from: "first", to: "last", data: {type: "KeepInRectangle", props: {x: toExpr(0, "m"), y: toExpr(0, "m"), w: toExpr(16.541, "m"), h: toExpr(8.0629, "m")}}, enabled: true}
        ],
        targetDt: toExpr(0.05, "s")
    },
    snapshot: { // ignore this
        waypoints: [],
        constraints: [],
        targetDt: 0.05
    },
    trajectory: {
        config: null,
        sampleType: undefined,
        waypoints: [],
        samples: [],
        splits: []
    },
    events: []
}


async function main() {
    const layoutName: string = await prompt("layout to generate: ");

    var layout: Layout = loadLayout(layoutName);
    var traj: Trajectory = buildTrajectory(baseTraj, layout.name, layout.layout);
    
    saveTrajectory(traj);

    console.log(`saved layout (${traj.name}):\n${JSON.stringify(traj, null, 2)}`)
    console.log(`\ngenerating trajectory`)

    generateTrajectory(traj);
}

main();
