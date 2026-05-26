import { ConstraintData } from "./choreo/ConstraintDefinitions";
import { Constraint, EventMarker, Expr, Trajectory, Waypoint } from "./choreo/DocumentTypes";

export function toExpr(val: number, unit: string): Expr {
    return { exp: `${val} ${unit}`, val: val };
}

export const FIELD_WIDTH = 8.0629;

export type Side = "l" | "c" | "r";
export type GenericLocation = "start" | "neutralbump" | "neutralmiddle" | "depot" | "human";

export type Location = `${Side}_${GenericLocation}`;

export type EventName = "pivot lower" | "pivot raise" | "lower depot enable" | "lower depot disable" | "feed in" | "feed stop" | "shoot" | "shoot still" | "stop shooting" | "extend";

export interface LocationProperties {
    leftWaypoints?: Waypoint<Expr>[],
    centerWaypoints?: Waypoint<Expr>[],
    rightWaypoints?: Waypoint<Expr>[],
    constraints?: Constraint[],
    eventMarkers?: EventMarker[]
}

export type AllLocationProperties = Record<GenericLocation, LocationProperties>;

// UPDATE AFTER FILE ADD/DELETE CHANGES TO all-location-properties
export type AllLocationPropertiesName = "snm" | "safe";

export interface Layout {
    name: string,
    allLocationProperties: AllLocationPropertiesName,
    layout: Location[],
    splitCommands: Record<number, EventName[]>
}

export const reflectWaypoints = (...waypoints: Waypoint<Expr>[]) => {
    return waypoints.map(waypoint => ({
        ...waypoint,
        y: toExpr(FIELD_WIDTH - waypoint.y.val, "m"),
        heading: toExpr(-waypoint.heading.val, "rad")
    }));
};

export const makeWaypoint = (x: number, y: number, heading: number, fixTranslation: boolean = true, fixHeading: boolean = true, split: boolean = false): Waypoint<Expr> => {
    return {
        x: toExpr(x, "m"),
        y: toExpr(y, "m"),
        heading: toExpr(heading, "rad"),
        intervals: 0,
        split: split,
        fixTranslation: fixTranslation,
        fixHeading: fixHeading,
        overrideIntervals: false
    };
};

export const makeConstraint = (from: number, to: number | undefined, data: ConstraintData): Constraint => {
    return {
        from: from,
        ...(to !== undefined && { to: to }),
        data: data,
        enabled: true
    };
}

export const makeEventMarker = (name: EventName, target: number, offset: number): EventMarker => {
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

/**
 * Shifts a {@link LocationProperties} by the given waypoint offset.
 * @param locationProperties The location properties.
 * @param waypointOffset The waypoint offset in the trajectory.
 * @returns New shifted location properties.
 */
export function shiftLocationProperties(locationProperties: LocationProperties, waypointOffset: number): LocationProperties {
    const clonedLocationProperties: LocationProperties = structuredClone(locationProperties);

    let shiftedConstraints: Constraint[] | undefined = undefined;
    let shiftedEventMarkers: EventMarker[] | undefined = undefined;

    // shift constraints if they exist
    if (locationProperties.constraints !== undefined) {
        shiftedConstraints = [];

        for (const constraint of locationProperties.constraints) {
            const shiftedConstraint: Constraint = structuredClone(constraint);

            shiftedConstraint.from = (constraint.from as number) + waypointOffset;

            if (shiftedConstraint.to == -1) {
                // connect constraint to the next waypoint
                shiftedConstraint.to = shiftedConstraint.from + 1;
            } else {
                shiftedConstraint.to = (constraint.to as number) + waypointOffset
            }

            shiftedConstraints.push(shiftedConstraint);
        }
    }

    // shift event markers if they exist
    if (locationProperties.eventMarkers !== undefined) {
        shiftedEventMarkers = [];

        for (const eventMarker of locationProperties.eventMarkers) {
            const shiftedEventMarker: EventMarker = structuredClone(eventMarker);

            shiftedEventMarker.from.target = (eventMarker.from.target as number) + waypointOffset;

            shiftedEventMarkers.push(shiftedEventMarker);
        }
    }

    return {
        ...clonedLocationProperties,
        constraints: shiftedConstraints,
        eventMarkers: shiftedEventMarkers
    }
}

/**
 * Stiches together a list of {@link Location}s to form an auto trajectory based off some given {@link AllLocationProperties}.
 * @param baseTraj The base trajectory to build off of.
 * @param name The name of the trajectory.
 * @param layout The list of locations.
 * @param allLocationProperties The set of location properties to use.
 * @returns Updated baseTraj.
 */
export function buildTrajectory(baseTraj: Trajectory, name: string, layout: Location[], allLocationProperties: AllLocationProperties): Trajectory {
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
