PARAMETER inclination IS 90.     //Target orbital inclination [degrees].
PARAMETER targetAlt IS 100000.   //Target orbital altitude [meters].

//====================================================================Phase 1 Functions====================================================================//

//Pitch Function
//Returns value of pitch for set altitude during launch and ascent.
SET slope1 TO (45 - 89) / (11000 - 1000).
SET slope2 TO (0 - 45) / (targetAlt - 11000).
FUNCTION pitchFunction {
    PARAMETER height.
        IF height <= 1000 {
            RETURN 89.
        } ELSE IF height <= 11000 {
            RETURN (slope1 * (height - 1000) + 89).
        } ELSE {
            RETURN (slope2 * (height - 11000) + 45).
        }
}

//Throttle Function
//Returns value for throttle controller during launch and ascent.
SET Kp TO 5.
FUNCTION throttleFunction {
    PARAMETER vesVelocity.
    PARAMETER vesAltitude.
    SET targetVelocity TO (114.37 / 1.0472) * CONSTANT:E^(0.121 * vesAltitude / 1000).
    IF MAXTHRUST > 0 {
        SET controller TO (MASS * Kp * (targetVelocity - vesVelocity) / MAXTHRUST).
    } ELSE {
        SET controller TO 0.
    }
    IF controller <= 0 {
        RETURN 0.
    } ELSE IF controller >= 1 {
        RETURN 1.
    } ELSE {
        RETURN controller.
    }
}

//Staging Function
//Stages if current stage is complete.
FUNCTION stagingFunction {
    SET readyToStage TO FALSE.
    SET noEnginesInStage TO TRUE.
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

//====================================================================Phase 2 Functions====================================================================//

//Correction Burn Throttle
//Returns value for throttle controller to fine tune apoapsis.
SET Kp2 TO 1/500.
FUNCTION correctionBurn {
    PARAMETER targetOrbit.
    PARAMETER currentApoapsis.
    SET controller TO Kp2*(targetOrbit - currentApoapsis).
    IF controller <= 0 {
        RETURN 0.
    } ELSE IF controller >= 1 {
        RETURN 1.
    } ELSE {
        RETURN controller.
    }
}

//====================================================================Phase 3 Functions====================================================================//

//Delta V Circularization Calculation
//Returns the delta V required for circularization at apoapsis.
FUNCTION circDeltaV {
    SET orbitVel TO SQRT(CONSTANT:G * KERBIN:MASS / (KERBIN:RADIUS + APOAPSIS)).
    SET apoapVel TO SQRT(CONSTANT:G * KERBIN:MASS * ((2 / (KERBIN:RADIUS + APOAPSIS)) - 1 / ((APOAPSIS + PERIAPSIS + 2 * KERBIN:RADIUS) / 2))).
    RETURN (orbitVel - apoapVel).
}


//Maneuver Node Throttle
//Returns throttle values to carry out maneuver node burns.
SET Kp3 TO 2.
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

//Maneuver Node Executor
//Executes next maneuver node accounting for possibility of requiring staging. Note that this function assumes only one engine in current and next stage.
FUNCTION executeNextManeuverNode {
    IF NOT HASNODE {
        RETURN.
    }

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

    SET stageFuelWeight TO 0.
    FOR res IN STAGE:RESOURCES {
        IF res:NAME = "LIQUIDFUEL" OR res:NAME = "OXIDIZER" {
            SET stageFuelWeight TO stageFuelWeight + res:AMOUNT * 5 / 1000.
        }
    }

    LIST ENGINES in engineList.
    SET ISP1 TO engineList[0]:VACUUMISP.

    SET currentStageDeltaV TO ISP1 * CONSTANT:g0 * LN(MASS / (MASS - stageFuelWeight)).

    IF currentStageDeltaV > NEXTNODE:DELTAV:MAG {
        SET finalMass1 TO MASS * CONSTANT:E ^ (-NEXTNODE:DELTAV:MAG / (ISP1 * CONSTANT:g0)).
        SET rateOfFuelConsumption1 TO -MAXTHRUST / (CONSTANT:g0 * ISP1).
        SET centroid1Time TO (MASS * CONSTANT:E ^ (-1/2 * LN(MASS / finalMass1)) - MASS) / rateOfFuelConsumption1.

        WARPTO(TIME:SECONDS + NEXTNODE:ETA - centroid1Time - 10).
        WAIT UNTIL NEXTNODE:ETA < centroid1Time.

        UNTIL NEXTNODE:DELTAV:MAG < 1 {
            LOCK THROTTLE TO maneuverNodeThrottler().
        }
    } ELSE {
        SET finalMass1 TO MASS - stageFuelWeight.
        SET rateOfFuelConsumption1 TO -MAXTHRUST / (CONSTANT:g0 * ISP1).
        SET centroid1Time TO (MASS * CONSTANT:E ^ (-1/2 * LN(MASS / finalMass1)) - MASS) / rateOfFuelConsumption1.
        SET deltaV1 TO ISP1 * CONSTANT:g0 * LN(MASS / finalMass1).
        SET burnTime1 TO (finalMass1 - MASS) / rateOfFuelConsumption1.

        SET initialMass2 TO MASS.
        SET ISP2 TO 0.

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

        SET finalMass2 TO initialMass2 * CONSTANT:E ^ (-NEXTNODE:DELTAV:MAG / (ISP2 * CONSTANT:g0)).
        SET rateOfFuelConsumption2 TO -MAXTHRUST / (CONSTANT:g0 * ISP2).
        SET centroid2Time TO (initialMass2 * CONSTANT:E ^ (-1/2 * LN(initialMass2 / finalMass2)) - initialMass2) / rateOfFuelConsumption2.
        SET deltaV2 TO ISP2 * CONSTANT:g0 * LN(initialMass2 / finalMass2).

        SET netCentroidTime TO (centroid1Time * deltaV1 + (burnTime1 + centroid2Time) * deltaV2) / (deltaV1 + deltaV2).

        WARPTO(TIME:SECONDS + NEXTNODE:ETA - netCentroidTime - 10).
        WAIT UNTIL NEXTNODE:ETA < netCentroidTime.

        UNTIL NEXTNODE:DELTAV:MAG < 0.1 {
            LOCK THROTTLE TO maneuverNodeThrottler().
            stagingFunction().
        }
    }
}

//==================================================================================================================================================================================================//
//====================================================================================Main Script===================================================================================================//
//==================================================================================================================================================================================================//

//Begin first stage.
STAGE.

//Phase 1: Launch into suborbital trajectory with apoapsis at target altitude. 

PRINT "PHASE 1: Initiated".
UNTIL (targetAlt - APOAPSIS) < 5000 {
    stagingFunction().
    SET targetHeading TO HEADING(inclination, pitchFunction(ALTITUDE)).
    LOCK STEERING TO targetHeading.
    SET throttleController TO throttleFunction(VELOCITY:SURFACE:MAG, ALTITUDE).
    LOCK THROTTLE TO throttleController.
}
LOCK THROTTLE TO 0.
PRINT "PHASE 1: Terminated".

//Phase 2: Apoapsis correction.

LOCK STEERING TO PROGRADE.
RCS ON.

IF ALTITUDE < 70000 {
    SET WARPMODE TO "PHYSICS".
    SET WARP TO 3.
}
WAIT UNTIL ALTITUDE > 70000.
SET WARP TO 0.
WAIT 1.
PRINT "PHASE 2: Initiated".
UNTIL (targetAlt - APOAPSIS) < 0 {
    SET throttleController TO correctionBurn(targetAlt + 20, APOAPSIS).
    LOCK THROTTLE TO throttleController.
}
LOCK THROTTLE TO 0.
PRINT "PHASE 2: Terminated".

//Phase 3: Orbit circularization.

PRINT "PHASE 3: Initiated".
SET deltaV TO circDeltaV().
SET circularizationBurn TO NODE(TIME:SECONDS + ETA:APOAPSIS, 0, 0, deltaV).
ADD circularizationBurn.
WAIT 1.
executeNextManeuverNode().
REMOVE circularizationBurn.
PRINT "PHASE 3: Terminated".
RCS OFF.