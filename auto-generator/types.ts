import { ConstraintData } from "./choreo/ConstraintDefinitions";
import { Constraint, EventMarker, Expr, Waypoint } from "./choreo/DocumentTypes";

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
export type AllLocationPropertiesName = "snm"

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