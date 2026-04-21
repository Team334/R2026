import { Expr, Constraint, Trajectory, TRAJ_SCHEMA_VERSION, Waypoint, EventMarker } from "./choreo/DocumentTypes";
import { AllLocationProperties, GenericLocation, Layout, Location, LocationProperties, shiftLocationProperties, Side, toExpr } from "./types";

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

function buildTrajectory(baseTraj: Trajectory, name: string, layout: Location[], allLocationProperties: AllLocationProperties): Trajectory {
    let trajWaypoints: Waypoint<Expr>[] = [];
    let trajConstraints: Constraint[] = [];
    let trajEventMarkers: EventMarker[] = [];

    let waypointOffset: number = 0;

    for (const location of layout) {
        const side = location.split('_')[0] as Side;

        const locationProperties: LocationProperties = shiftLocationProperties(
            allLocationProperties[location.split('_')[1] as GenericLocation],
            waypointOffset
        );

        let locationWaypoints: Waypoint<Expr>[] = [];

        switch (side) {
            case "l":
                locationWaypoints = locationProperties.leftWaypoints ?? [];
                break;

            case "c":
                locationWaypoints = locationProperties.centerWaypoints ?? [];
                break;

            case "r":
                locationWaypoints = locationProperties.rightWaypoints ?? [];
                break;

            default:
                break;
        }

        trajWaypoints.push(...locationWaypoints);
        trajConstraints.push(...(locationProperties.constraints ?? []));
        trajEventMarkers.push(...(locationProperties.eventMarkers ?? []));

        waypointOffset += locationWaypoints.length
    }

    const updatedTraj: Trajectory = baseTraj;

    updatedTraj.name = name;

    updatedTraj.params.waypoints = trajWaypoints;
    updatedTraj.params.constraints.push(...trajConstraints);
    updatedTraj.events.push(...trajEventMarkers);

    return updatedTraj;
}

var baseTraj: Trajectory = {
    name: "",
    version: TRAJ_SCHEMA_VERSION,
    params: {
        waypoints: [],
        constraints: [
            { from: "first", data: { type: "StopPoint", props: {} }, enabled: true },
            { from: "last", data: { type: "StopPoint", props: {} }, enabled: true },
            { from: "first", to: "last", data: { type: "MaxVelocity", props: { max: toExpr(2.5, "m/s") } }, enabled: true },
            { from: "first", to: "last", data: { type: "MaxAcceleration", props: { max: toExpr(3, "m/s^2") } }, enabled: true },
            { from: "first", to: "last", data: { type: "KeepOutCircle", props: { x: toExpr(4.66, "m"), y: toExpr(6.18, "m"), r: toExpr(0.826396949005302, "m") } }, enabled: true },
            { from: "first", to: "last", data: { type: "KeepOutCircle", props: { x: toExpr(4.66, "m"), y: toExpr(1.8892, "m"), r: toExpr(0.826396949005302, "m") } }, enabled: true },
            { from: "first", to: "last", data: { type: "KeepInRectangle", props: { x: toExpr(0, "m"), y: toExpr(0.0392, "m"), w: toExpr(16.541, "m"), h: toExpr(7.9908, "m") } }, enabled: true }
        ],
        targetDt: toExpr(0.05, "s")
    },
    snapshot: { // ignore
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

    var traj: Trajectory = buildTrajectory(baseTraj, layout.name, layout.layout, allLocationProperties);

    saveTrajectory(traj);

    console.log(`\nconverted layout into ${traj.name}.traj`)
    console.log(`now generating ${traj.name}.traj`)

    generateTrajectory(traj);
}

main();
