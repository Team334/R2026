import { Expr, Constraint, Trajectory, TRAJ_SCHEMA_VERSION, Waypoint } from "./choreo/DocumentTypes";

// @ts-ignore
import * as fs from "fs";

// @ts-ignore
import { exec } from "child_process";

// @ts-ignore
import os from "os";
import { ConstraintData } from "./choreo/ConstraintDefinitions";

const CHOREO_CLI = os.homedir().replace(/\\/g, "/") + '/AppData/Local/choreo/choreo-cli';

// (assume running generator from R2026 dir)
const DEPLOY_DIR: string = './src/main/deploy';
const TRAJ_DIR: string = DEPLOY_DIR + '/choreo/';

const LAYOUT_DIR: string = DEPLOY_DIR + '/layouts';
const CONFIG: string = TRAJ_DIR + '/config.chor';

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

function shiftConstraints(constraints: Constraint[], waypointOffset: number) : Constraint[] {
    const shiftedConstraints: Constraint[] = [];

    for (const constraint of constraints) {
        if (typeof constraint.from === 'number') constraint.from += waypointOffset;
        if (typeof constraint.to === 'number') constraint.to += waypointOffset;

        shiftedConstraints.push(constraint);
    }

    return shiftedConstraints;
}

function buildTrajectory(baseTraj: Trajectory, name: string, side: Side, layout: Location[]) : Trajectory {
    let trajWaypoints: Waypoint<Expr>[] = [];
    let trajConstraints: Constraint[] = [];

    let waypointOffset: number = 0;

    for (const location of layout) {
        const params = locationParams[location];

        switch (side) {
            case "L":
                trajWaypoints.push(...(params.leftWaypoints ?? []));
                trajConstraints.push(...shiftConstraints(params.constraints ?? [], waypointOffset));

                waypointOffset += params.leftWaypoints?.length ?? 0;
                break;
    
            case "C":
                trajWaypoints.push(...(params.centerWaypoints ?? []));
                trajConstraints.push(...shiftConstraints(params.constraints ?? [], waypointOffset));

                waypointOffset += params.centerWaypoints?.length ?? 0;
                break;

            case "R":
                trajWaypoints.push(...(params.rightWaypoints ?? []));
                trajConstraints.push(...shiftConstraints(params.constraints ?? [], waypointOffset));

                waypointOffset += params.rightWaypoints?.length ?? 0;
                break;

            default:
                break;
        }
    }

    const updatedTraj: Trajectory = baseTraj;

    updatedTraj.name = side + name;

    updatedTraj.params.waypoints = trajWaypoints;
    updatedTraj.params.constraints.push(...trajConstraints);

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

type Location = "start" | "neutralbump" | "neutralmiddle" | "trench" | "depot" | "human" | "climb";
type Side = "L" | "C" | "R";

interface LocationParams {
    leftWaypoints?: Waypoint<Expr>[],
    centerWaypoints?: Waypoint<Expr>[],
    rightWaypoints?: Waypoint<Expr>[],
    constraints?: Constraint[] // assume no coordinate constraints per location
}

interface Layout {
    name: string,
    excludedSides: Side[],
    layout: Location[]
}

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

const locationParams: Record<Location, LocationParams> = {
    start: {
        centerWaypoints: [makeWaypoint(3, 4, 0)]
    },
    neutralbump: {},
    neutralmiddle: {
        centerWaypoints: [
            makeWaypoint(4.059676647186279, 7.521993637084961, -1.5599661588553948),
            makeWaypoint(7.636203765869141, 6.131853103637695, -1.4711286226200226),
            makeWaypoint(7.501785278320312, 4.563638210296631, -2.0576957311828057),
            makeWaypoint(4.586862087249756, 7.546622276306152, -1.5708)
        ]
    },
    trench: {},
    depot: {
        centerWaypoints: [
            makeWaypoint(0.4917987287044525, 6.870540618896484, -1.5707963267948966),
            makeWaypoint(0.4917987287044525, 5.075254440307617, -1.5707963267948966)
        ],
        constraints: [
            makeConstraint(0, 1, {type: "KeepInLane", props: {tolerance: toExpr(0.03, "m")}})
        ]
    },
    human: {},
    climb: {}
}

var baseTraj: Trajectory = {
    name: "",
    version: TRAJ_SCHEMA_VERSION,
    params: {
        waypoints: [],
        constraints: [
            {from: "first", data: {type: "StopPoint", props: {}}, enabled: true},
            {from: "last", data: {type: "StopPoint", props: {}}, enabled: true},
            {from: "first", to: "last", data: {type: "KeepOutCircle", props: {x: toExpr(4.6220447067171335, "m"), y: toExpr(6.499967720359564, "m"), r: toExpr(0.7, "m")}}, enabled: true},
            // TODO right side keep out circle
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

var layout: Layout = loadLayout("layout");

var traj : Trajectory = buildTrajectory(baseTraj, layout.name, "C", layout.layout);

saveTrajectory(traj);
generateTrajectory(traj);
