//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
// Adapted for Low Level Parallel Programming 2017
//
// TAgent represents an agent in the scenario. Each
// agent has a position (x,y) and a number of destinations
// it wants to visit (waypoints). The desired next position
// represents the position it would like to visit next as it
// will bring it closer to its destination.
// Note: the agent will not move by itself, but the movement
// is handled in ped_model.cpp.
//

#include <cstddef>
#ifndef _ped_agent_h_
#define _ped_agent_h_ 1

#include <deque>
#include <vector>
#include <atomic>

using namespace std;

namespace Ped
{
    class Twaypoint;

    class Tagent
    {
    public:
        Tagent(void **positionArrays, int agentIndex);

        // Returns the coordinates of the desired position
        int getDesiredX() const { return *desiredPositionX; }
        int getDesiredY() const { return *desiredPositionY; }

        // Sets the agent's position
        void setX(int newX) { *x = newX; }
        void setY(int newY) { *y = newY; }

        // Update the position according to get closer
        // to the current destination
        void computeNextDesiredPosition();

        // Returns the next destination to visit
        void setNextDestination();

        // Position of agent defined by x and y
        int getX() const { return *x; };
        int getY() const { return *y; };

        // Adds a new waypoint to reach for this agent
        void addWaypoint(Twaypoint *wp);

        // gets the next agent
        Tagent *getNextAgent();

        // gets a pointer to the nextAgent field
        std::atomic<Ped::Tagent *> *getNextAgentField();

        // sets the next agent to the agent chosen
        Tagent *setNextAgent(Tagent *new_agent);

        bool getHasMoved() { return hasMoved; }
        void setHasMoved(bool hasMoved) { hasMoved = hasMoved; }

    private:
        Tagent() {};

        // The agent's current position
        int *x;
        int *y;

        // The agent's desired next position
        int *desiredPositionX;
        int *desiredPositionY;

        double *destinationPosX;
        double *destinationPosY;

        // The current destination (may require several steps to reach)
        Twaypoint *destination;

        // The last destination
        Twaypoint *lastDestination;

        // The queue of all destinations that this agent still has to visit
        deque<Twaypoint *> waypoints;

        // Internal init function
        void init(void **positionArrays, int agentIdex);

        // next agent in list
        std::atomic<Ped::Tagent *> next_agent{NULL};

        bool hasMoved = false;
    };
} // namespace Ped

#endif
