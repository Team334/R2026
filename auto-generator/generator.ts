import { Expr, Constraint, Trajectory, TRAJ_SCHEMA_VERSION, Waypoint, EventMarker } from "./choreo/DocumentTypes";
import { AllLocationProperties, buildTrajectory, FIELD_WIDTH, GenericLocation, Layout, Location, LocationProperties, shiftLocationProperties, Side, toExpr } from "./types";

// @ts-ignore
import * as fs from "fs";

// @ts-ignore
import { exec } from "child_process";

// @ts-ignore
import os from "os";

// @ts-ignore
import readline from "readline/promises";

const CHOREO_CLI = `${os.homedir().replace(/\\/g, "/")}/AppData/Local/choreo/choreo-cli`;

// (assume running generator from R2026 dir)
const DEPLOY_DIR: string = './src/main/deploy';
const TRAJ_DIR: string = `${DEPLOY_DIR}/choreo/`;

const LAYOUT_DIR: string = `${DEPLOY_DIR}/layouts`;
const CONFIG: string = `${TRAJ_DIR}/config.chor`;

const rl = readline.createInterface({
    // @ts-ignore
    input: process.stdin,

    // @ts-ignore
    output: process.stdout
});

async function prompt(q: string) {
    return await rl.question(q);
}

function loadLayout(name: string): Layout {
    const layoutData = fs.readFileSync(`${LAYOUT_DIR}/${name}.json`, 'utf-8');

    return JSON.parse(layoutData) as Layout;
}

async function loadAllLocationProperties(name: string): Promise<AllLocationProperties> {
    const file = await import(`./all-location-properties/${name}.ts`);
    return file.allLocationProperties as AllLocationProperties;
}

function saveTrajectory(traj: Trajectory): void {
    fs.writeFileSync(`${TRAJ_DIR}/${traj.name}.traj`, JSON.stringify(traj, null, 2));
}

function generateTrajectory(traj: Trajectory): void {
    exec(`${CHOREO_CLI} --chor ${CONFIG} --trajectory ${traj.name}.traj -g`, (error: Error | null, stdout: string, stderr: string) => {
        if (error) {
            console.error("choreo cli:", error);
            return;
        }

        console.log("choreo cli:");
        console.log(stdout);
    });
}

var rebuiltTraj: Trajectory = {
    name: "",
    version: TRAJ_SCHEMA_VERSION,
    params: {
        waypoints: [],
        constraints: [
            { from: "first", data: { type: "StopPoint", props: {} }, enabled: true },
            { from: "last", data: { type: "StopPoint", props: {} }, enabled: true },
            { from: "first", to: "last", data: { type: "MaxVelocity", props: { max: toExpr(3.5, "m/s") } }, enabled: true },
            { from: "first", to: "last", data: { type: "MaxAcceleration", props: { max: toExpr(3.5, "m/s^2") } }, enabled: true },
            { from: "first", to: "last", data: { type: "KeepOutCircle", props: { x: toExpr(4.612211856842041, "m"), y: toExpr(6.084839515686035, "m"), r: toExpr(0.9316006363189093, "m") } }, enabled: true },
            { from: "first", to: "last", data: { type: "KeepOutCircle", props: { x: toExpr(4.612211856842041, "m"), y: toExpr(FIELD_WIDTH - 6.084839515686035, "m"), r: toExpr(0.9316006363189093, "m") } }, enabled: true },
            { from: "first", to: "last", data: { type: "KeepInRectangle", props: { x: toExpr(0, "m"), y: toExpr(0.0292, "m"), w: toExpr(16.541, "m"), h: toExpr(8.0008, "m") } }, enabled: true }
        ],
        targetDt: toExpr(0.05, "s")
    },
    snapshot: {
        waypoints: [],
        constraints: [],
        targetDt: 0.05
    },
    trajectory: {
        config: null,
        sampleType: "Swerve",
        waypoints: [],
        samples: [],
        splits: []
    },
    events: []
}

async function main() {
    var layout: Layout = loadLayout(await prompt("layout to generate: "));
    var allLocationProperties: AllLocationProperties = await loadAllLocationProperties(layout.allLocationProperties);

    var traj: Trajectory = buildTrajectory(rebuiltTraj, layout.name, layout.layout, allLocationProperties);

    saveTrajectory(traj);

    console.log(`\nconverted layout into ${traj.name}.traj`)

    var generate: string = await prompt("proceed to choreo generation? (y/n): ")

    if (generate === "y") {
        console.log(`now generating ${traj.name}.traj`)
        generateTrajectory(traj);
        return;
    }

    console.log(`ok`);
}

main();
