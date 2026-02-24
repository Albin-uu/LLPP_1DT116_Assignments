//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <cstdio>
#include <math.h>

#include <pthread.h>
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
	destinationPosX = ((double *)positionArrays[2]) + agentIndex;
	destinationPosY = ((double *)positionArrays[3]) + agentIndex;
	desiredPositionX = ((int *)positionArrays[4]) + agentIndex;
	desiredPositionY = ((int *)positionArrays[5]) + agentIndex;
	destination = NULL;
	lastDestination = NULL;
	next_agent = NULL;
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
	// std::cout << "iiii" << std::endl;

	double diffX = *destinationPosX - *x;
	double diffY = *destinationPosY - *y;
	double len = sqrt(diffX * diffX + diffY * diffY);
	if (len == 0) // Handle if agent is already at goal, avoid divide by 0.
	{
		*desiredPositionX = *this->x;
		*desiredPositionY = *this->y;
	}
	else
	{
		*desiredPositionX = (int)round(*x + diffX / len);
		*desiredPositionY = (int)round(*y + diffY / len);
	}
	// printf("x: %d\n", *x);
	// printf("diff: %f\n", diffX);
	// printf("len: %f\n", len);
	// printf("desiredpos: %d\n", *desiredPositionX);
}

Ped::Tagent *Ped::Tagent::getNextAgent()
{
	return this->next_agent;
}

// gets a pointer to the nextAgent field
Ped::Tagent **Ped::Tagent::getNextAgentField()
{
	return &this->next_agent;
}

// sets the next agent to the agent chosen
Ped::Tagent *Ped::Tagent::setNextAgent(Tagent *next_agent)
{
	this->next_agent = next_agent;
	return next_agent;
}

void Ped::Tagent::addWaypoint(Twaypoint *wp) { waypoints.push_back(wp); }

void Ped::Tagent::setNextDestination()
{
	bool agentReachedDestination = false;

	if (destination != NULL)
	{
		// compute if agent reached its current destination
		// std::cout << "pre diff" << std::endl;
		double diffX = *destinationPosX - *x;
		double diffY = *destinationPosY - *y;
		// std::cout << "diff done" << std::endl;
		double length = sqrt(diffX * diffX + diffY * diffY);
		agentReachedDestination = length < destination->getr();
	}

	if ((agentReachedDestination || destination == NULL) && !waypoints.empty())
	{
		// std::cout << "aaaa" << std::endl;
		//  Case 1: agent has reached destination (or has no current destination);
		//  get next destination if available
		if (destination != NULL)
		{
			waypoints.push_back(destination);
		}
		destination = waypoints.front();
		*destinationPosX = destination->getx();
		*destinationPosY = destination->gety();
		waypoints.pop_front();
	}
}
