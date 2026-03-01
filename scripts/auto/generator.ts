import { Expr, Constraint, Trajectory, TRAJ_SCHEMA_VERSION, Waypoint } from "./choreo/DocumentTypes";

// @ts-ignore
import * as fs from "fs";

// @ts-ignore
import { exec } from "child_process";

// @ts-ignore
import os from "os";

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

function buildTrajectory(baseTraj: Trajectory, name: string, side: Side, layout: Location[]) : Trajectory {
    let trajWaypoints: Waypoint<Expr>[] = [];

    for (const location of layout) {
        const params = locationParams[location];

        switch (side) {
            case "L":
                trajWaypoints.push(...(params.leftWaypoints ?? []));
                break;
    
            case "C":
                trajWaypoints.push(...(params.centerWaypoints ?? []));
                break;

            case "R":
                trajWaypoints.push(...(params.rightWaypoints ?? []));
                break;

            default:
                break;
        }
    }

    baseTraj.params.waypoints = trajWaypoints;
    baseTraj.name = side + name;

    return baseTraj;
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

const locationParams: Record<Location, LocationParams> = {
    start: {
        centerWaypoints: [makeWaypoint(3, 4, 0)]
    },
    neutralbump: {},
    neutralmiddle: {
        centerWaypoints: [
            makeWaypoint(4.125550270080566, 7.580191135406494, -1.5599661588553948),
            makeWaypoint(7.636203765869141, 6.131853103637695, -1.4711286226200226),
            makeWaypoint(7.501785278320312, 4.563638210296631, -2.0576957311828057),
            makeWaypoint(4.662569522857666, 7.593451023101807, 0)
        ]
    },
    trench: {},
    depot: {
        centerWaypoints: [
            makeWaypoint(0.3876773416996002, 6.857964038848877, -1.5707963267948966)
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
        constraints: [],
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

// console.log(traj.params.waypoints.length);

saveTrajectory(traj);
generateTrajectory(traj);