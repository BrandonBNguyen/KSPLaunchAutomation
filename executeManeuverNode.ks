PARAMETER tolerance IS 0.2.

LOCK STEERING TO NEXTNODE:DELTAV.
SET facingIsClose TO FALSE.
UNTIL facingIsClose {
    SET pitchDiff TO ABS(FACING:PITCH - NEXTNODE:DELTAV:DIRECTION:PITCH).
    SET rollDiff TO ABS(FACING:ROLL - NEXTNODE:DELTAV:DIRECTION:ROLL).
    SET yawDiff TO ABS(FACING:YAW - NEXTNODE:DELTAV:DIRECTION:YAW).
    IF ((pitchDiff < 0.5 OR pitchDiff > (360-0.5)) AND (rollDiff < 0.5 OR rollDiff > (360-0.5))) 
    OR ((pitchDiff < 0.5 OR pitchDiff > (360-0.5)) AND (yawDiff < 0.5 OR yawDiff > (360-0.5)))
    OR ((rollDiff < 0.5 OR rollDiff > (360-0.5)) AND (yawDiff < 0.5 OR yawDiff > (360-0.5))) 
    {
        SET facingIsClose TO TRUE.
    }
}

DECLARE Kp3 TO 2.
FUNCTION maneuverNodeThrottler {
    SET maxAcceleration TO MAXTHRUST / MASS.
    IF maxAcceleration = 0 {
        RETURN 0.
    }
    SET controller TO Kp3*(NEXTNODE:DELTAV:MAG / maxAcceleration).
    IF controller <= 0 {
        RETURN 0.
    } ELSE IF controller >= 1 {
        RETURN 1.
    } ELSE {
        RETURN controller.
    }
}

FUNCTION stagingFunction {
    DECLARE readyToStage TO FALSE.
    DECLARE noEnginesInStage TO TRUE.
    LIST ENGINES IN elist.
    FOR e IN elist {
        IF e:FLAMEOUT {
            SET readyToStage TO TRUE.
        }

        IF e:IGNITION {
            SET noEnginesInStage TO FALSE.
        }
    }
    IF (readyToStage OR noEnginesInStage) {
        STAGE.

        UNTIL STAGE:READY {
                WAIT 0.
            }

        RETURN STAGE.
    }

}

DECLARE stageFuelWeight TO 0.
FOR res IN STAGE:RESOURCES {
    IF res:NAME = "LIQUIDFUEL" OR res:NAME = "OXIDIZER" {
        SET stageFuelWeight TO stageFuelWeight + res:AMOUNT * 5 / 1000.
    }
}

LIST ENGINES in engineList.
DECLARE ISP1 TO engineList[0]:VACUUMISP.

DECLARE currentStageDeltaV TO ISP1 * CONSTANT:g0 * LN(MASS / (MASS - stageFuelWeight)).

IF currentStageDeltaV > NEXTNODE:DELTAV:MAG {
    DECLARE finalMass1 TO MASS * CONSTANT:E ^ (-NEXTNODE:DELTAV:MAG / (ISP1 * CONSTANT:g0)).
    DECLARE rateOfFuelConsumption1 TO -MAXTHRUST / (CONSTANT:g0 * ISP1).
    DECLARE centroid1Time TO (MASS * CONSTANT:E ^ (-1/2 * LN(MASS / finalMass1)) - MASS) / rateOfFuelConsumption1.

    WARPTO(TIME:SECONDS + NEXTNODE:ETA - centroid1Time - 10).
    WAIT UNTIL NEXTNODE:ETA < centroid1Time.

    UNTIL NEXTNODE:DELTAV:MAG < tolerance {
        LOCK THROTTLE TO maneuverNodeThrottler().
    }
} ELSE {
    DECLARE finalMass1 TO MASS - stageFuelWeight.
    DECLARE rateOfFuelConsumption1 TO -MAXTHRUST / (CONSTANT:g0 * ISP1).
    DECLARE centroid1Time TO (MASS * CONSTANT:E ^ (-1/2 * LN(MASS / finalMass1)) - MASS) / rateOfFuelConsumption1.
    DECLARE deltaV1 TO ISP1 * CONSTANT:g0 * LN(MASS / finalMass1).
    DECLARE burnTime1 TO (finalMass1 - MASS) / rateOfFuelConsumption1.

    DECLARE initialMass2 TO MASS.
    SET ISP2 TO 0.
    DECLARE checkThisStage TO STAGE:NUMBER - 2.

    LIST PARTS in partsList.
    FOR part IN partsList {
        IF part:STAGE = STAGE:NUMBER {
            SET initialMass2 TO initialMass2 - part:MASS.
        }

        SET checkThisStage TO STAGE:NUMBER - 2.
        UNTIL NOT (ISP2 = 0) OR checkThisStage < 0 {
            IF part:NAME:CONTAINS("engine") AND part:STAGE = checkThisStage {
                SET ISP2 TO part:VACUUMISP.
            } ELSE {
            SET checkThisStage TO checkThisStage - 1.
            }
        }
    }

    DECLARE finalMass2 TO initialMass2 * CONSTANT:E ^ (-NEXTNODE:DELTAV:MAG / (ISP2 * CONSTANT:g0)).
    DECLARE rateOfFuelConsumption2 TO -MAXTHRUST / (CONSTANT:g0 * ISP2).
    DECLARE centroid2Time TO (initialMass2 * CONSTANT:E ^ (-1/2 * LN(initialMass2 / finalMass2)) - initialMass2) / rateOfFuelConsumption2.
    DECLARE deltaV2 TO ISP2 * CONSTANT:g0 * LN(initialMass2 / finalMass2).

    DECLARE netCentroidTime TO (centroid1Time * deltaV1 + (burnTime1 + centroid2Time) * deltaV2) / (deltaV1 + deltaV2).

    WARPTO(TIME:SECONDS + NEXTNODE:ETA - netCentroidTime - 10).
    WAIT UNTIL NEXTNODE:ETA < netCentroidTime.

    UNTIL NEXTNODE:DELTAV:MAG < tolerance {
        LOCK THROTTLE TO maneuverNodeThrottler().
        stagingFunction().
    }
}

REMOVE NEXTNODE.