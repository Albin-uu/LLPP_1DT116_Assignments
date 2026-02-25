

#include "ped_agent.h"
#include <mutex>
#include <string>
#include <atomic>
#include <cstdint>

#define REGION_HEIGHT 10 // temp width macro

namespace Ped
{
    class Tregion
    {
    public:
        Tregion(int id) { this->id = id; };

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

        // move ownership of current agent to another region
        void moveCurrentToAnotherRegion(Tregion *region);

        // finds out whether position is in this region
        // bool isInRegion(int x, int y);

        // append
        void append(Ped::Tagent *agent);

    private:
        Ped::Tagent *pop();
        string get_uuid();

        // int greaterRegion;
        // int xStart;
        // int xEnd;
        // const string id = get_uuid();
        int id;

        Ped::Tagent *current = NULL;
        std::atomic<Ped::Tagent *> *previous = NULL;

        std::mutex agentsLock;
        std::atomic<Ped::Tagent *> startAgent = NULL;
        std::atomic<std::atomic<Ped::Tagent *> *> endField{&startAgent};
    };
}
