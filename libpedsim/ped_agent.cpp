//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <math.h>

#include <stdlib.h>

Ped::Tagent::Tagent(void **positionArrays,
                    int agentIdex)
{
    Ped::Tagent::init(positionArrays, agentIdex);
}

void Ped::Tagent::init(void **positionArrays,
                       int agentIndex)
{
    x = ((int *)positionArrays[0]) + agentIndex;
    y = ((int *)positionArrays[1]) + agentIndex;
    desiredPositionX = ((int *)positionArrays[2]) + agentIndex;
    desiredPositionY = ((int *)positionArrays[3]) + agentIndex;
    destinationPosX = ((double *)positionArrays[4]) + agentIndex;
    destinationPosY = ((double *)positionArrays[5]) + agentIndex;
    destination = NULL;
    lastDestination = NULL;
}

void Ped::Tagent::computeNextDesiredPosition()
{
    setNextDestination();
    if (destination == NULL)
    {
        // no destination, no need to
        // compute where to move to
        return;
    }

    double diffX = *desiredPositionX - *x;
    double diffY = *desiredPositionY - *y;
    double len = sqrt(diffX * diffX + diffY * diffY);
    *desiredPositionX = (int)round(*x + diffX / len);
    *desiredPositionY = (int)round(*y + diffY / len);
}

void Ped::Tagent::addWaypoint(Twaypoint *wp) { waypoints.push_back(wp); }

void Ped::Tagent::setNextDestination()
{
    Ped::Twaypoint *nextDestination = NULL;
    bool agentReachedDestination = false;

    if (destination != NULL)
    {
        // compute if agent reached its current destination
        double diffX = *desiredPositionX - *x;
        double diffY = *desiredPositionY - *y;
        double length = sqrt(diffX * diffX + diffY * diffY);
        agentReachedDestination = length < destination->getr();
    }

    if ((agentReachedDestination || destination == NULL) && !waypoints.empty())
    {
        // Case 1: agent has reached destination (or has no current destination);
        // get next destination if available
        waypoints.push_back(destination);
        destination = waypoints.front();
        *destinationPosX = destination->getx();
        *destinationPosY = destination->gety();
        waypoints.pop_front();
    }
}
