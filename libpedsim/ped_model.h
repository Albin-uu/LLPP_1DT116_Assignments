//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
// Adapted for Low Level Parallel Programming 2017
//
// Model coordinates a time step in a scenario: for each
// time step all agents need to be moved by one position if
// possible.
//
#ifndef _ped_model_h_
#define _ped_model_h_

#include <map>
#include <mutex>
#include <set>
#include <vector>

#include "ped_agent.h"
#include "ped_region.h"
#include "positionTracker.h"

namespace Ped
{
    class Tagent;

    // The implementation modes for Assignment 1 + 2:
    // chooses which implementation to use for tick()
    enum IMPLEMENTATION
    {
        CUDA,
        VECTOR,
        OMP,
        PTHREAD,
        SEQ,
        COLLISION_SEQ,
        COLLISION_OMP,
        COLLISION_OMP_SIMD,
        COLLISION_OMP_HEATMAP
    };

    class Model
    {
    public:
        // Sets everything up
        void setup(std::vector<Tagent *> agentsInScenario,
                   std::vector<Twaypoint *> destinationsInScenario,
                   void **positionArrays,
                   IMPLEMENTATION implementation);

        // Coordinates a time step in the scenario: move all agents by one step (if
        // applicable).
        void tick();

        // Returns the agents of this scenario
        const std::vector<Tagent *> &getAgents() const { return agents; };

        // Adds an agent to the tree structure
        void placeAgent(const Ped::Tagent *a);

        void freePosArrs();

        Ped::Tregion *calculateRegion(int x, int y);

        // Cleans up the tree and restructures it. Worth calling every now and then.
        void cleanup();
        ~Model();

        // Returns the heatmap visualizing the density of agents
        int const *const *getHeatmap() const { return blurred_heatmap; };
        int getHeatmapSize() const;

    private:
        std::vector<Ped::Tregion *> collectRegions();

        // Denotes which implementation (sequential, parallel implementations..)
        // should be used for calculating the desired positions of
        // agents (Assignment 1)
        IMPLEMENTATION implementation;

        // The agents in this scenario
        std::vector<Tagent *> agents;

        // The waypoints in this scenario
        std::vector<Twaypoint *> destinations;

        // Regions for collision versions.
        std::vector<
            std::map<int, std::pair<Ped::Tregion *, bool>>>
            regions;

        // Position bitmasks array
        Ped::Tpositions positionTracker;

        // Used for cuda.
        int *agentsDesiredY;
        int *agentsDesiredX;

        int *agentX;
        int *agentY;
        double *destinationX;
        double *destinationY;
        int *desiredX;
        int *desiredY;

        int ticks = 0;

        // Moves an agent towards its next position
        void move(Ped::Tagent *agent);

        void sequentialTick();
        void simdTick();
        void simdCompNextDesiredPos();
        void ompTick();
        void cppTick();
        void collisionSequentialTick();
        void collisionOMPTick();
        void collisionOMPSIMDTick();
        void collisionOMPHMTick();

        void dynamicResizeRegions();
        void parrallelCMove();
        ////////////
        /// Everything below here won't be relevant until Assignment 3
        ///////////////////////////////////////////////

        // Returns the set of neighboring agents for the specified position
        set<const Ped::Tagent *> getNeighbors(int x, int y, int dist) const;

        ////////////
        /// Everything below here won't be relevant until Assignment 4
        ///////////////////////////////////////////////

#define SIZE 1024
#define CELLSIZE 5
#define SCALED_SIZE SIZE *CELLSIZE

        // The heatmap representing the density of agents
        int **heatmap;

        // The scaled heatmap that fits to the view
        int **scaled_heatmap;

        // The final heatmap: blurred and scaled to fit the view
        int **blurred_heatmap;

        void setupHeatmapSeq();
        void setupHeatmapCuda();
        void updateHeatmapSeq();
        void updateHeatmapCuda(int *agentsDesiredX, int *agentsDesiredY, int agentCount);
    };
} // namespace Ped
#endif
