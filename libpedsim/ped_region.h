

#include "ped_agent.h"
#include <mutex>
#include <string>
#include <atomic>
#include <cstdint>

#define REGION_HEIGHT 10 // temp width macro

#define REGION_ID_Y (255 << 8)
#define REGION_ID_X (255)

namespace Ped
{
    class Tregion
    {
    public:
        Tregion(int id, int xEnd)
        {
            this->id = id;
            this->xEnd = xEnd;
        };

        // gets the regions id
        // string getId() {return this->id;};
        int getId()
        {
            return this->id;
        };

        // moves on to the next agent and gets it
        Ped::Tagent *getNext();

        // Initialises the iterator and gets the start
        Ped::Tagent *getStart();

        // move ownership of current agent to another region and return the new agent at the current position
        Ped::Tagent *moveCurrentToAnotherRegion(Tregion *region);

        // finds out whether position is in this region
        // bool isInRegion(int x, int y);

        // append
        void append(Ped::Tagent *agent);

        int getAmountOfAgents() { return amountOfAgents; }

        // splits the region in two and moves half of the agents to the new region
        // returns the x value of the split line
        Ped::Tregion *splitRegion();

        void mergeRegion(Ped::Tregion *otherRegion);

        int getXStart() { return id & REGION_ID_X; }
        int getXEnd() { return xEnd; }
        int getSlice() { return id >> 8; }

    private:
        int findMedianX();

        Ped::Tagent *pop();
        string get_uuid();

        atomic_int amountOfAgents{0};

        // int greaterRegion;
        int xEnd;
        // const string id = get_uuid();
        int id;

        Ped::Tagent *current = NULL;
        std::atomic<Ped::Tagent *> *previous = NULL;

        std::atomic<Ped::Tagent *> startAgent{NULL};
        std::atomic<std::atomic<Ped::Tagent *> *> endField{&startAgent};
    };
}
