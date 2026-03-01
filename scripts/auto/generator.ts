import { Expr, Constraint, Trajectory, TRAJ_SCHEMA_VERSION } from "./choreo/DocumentTypes";

// @ts-ignore
import * as fs from "fs";

// @ts-ignore
import { exec } from "child_process";

// @ts-ignore
import os from "os";

const CHOREO_CLI = os.homedir().replace(/\\/g, "/") + '/AppData/Local/choreo/choreo-cli';

// (assume running generator from R2026 dir)
const TRAJ_DIR: string = './src/main/deploy/choreo/';
const CONFIG: string = TRAJ_DIR + 'config.chor';

function toExpr(val: number, unit: string) : Expr {
  return { exp: `${val} ${unit}`, val: val };
}

/**
 * Save trajectory to deploy/choreo.
 */
function saveTrajectory(traj: Trajectory) {
    fs.writeFileSync(TRAJ_DIR + traj.name + ".traj", JSON.stringify(traj, null, 2));
}

/**
 * Generate the trajectory through choreo cli. The trajectory must exist in deploy/choreo first.
 */
function generateTrajectory(traj: Trajectory) {
    exec(CHOREO_CLI + " --chor " + CONFIG + " --trajectory " + traj.name + ".traj" + " -g", (error: Error | null, stdout: string, stderr: string) => {
        if (error) {
            console.error("Choreo Error:", error);
            return;
        }

        console.log("Choreo Output:");
        console.log(stdout);
    });
}

var baseTraj: Trajectory = {
    name: "NewTrajectory",
    version: TRAJ_SCHEMA_VERSION,
    params: {
        waypoints: [
            {x: toExpr(2, "m"), y: toExpr(5, "m"), heading: toExpr(0, "rad"), intervals:0, split:false, fixTranslation:true, fixHeading:true, overrideIntervals:false},
            {x: toExpr(6, "m"), y: toExpr(5, "m"), heading: toExpr(0, "rad"), intervals:0, split:false, fixTranslation:true, fixHeading:true, overrideIntervals:false},
            {x: toExpr(4, "m"), y: toExpr(3, "m"), heading: toExpr(0, "rad"), intervals:0, split:false, fixTranslation:true, fixHeading:true, overrideIntervals:false}
        ],
        constraints: [],
        targetDt: toExpr(0.05, "s")
    },
    snapshot: {
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

saveTrajectory(baseTraj);
generateTrajectory(baseTraj);