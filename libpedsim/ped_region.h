

#include "ped_agent.h"
#include <mutex>
namespace Ped
{
    class Tregion
    {
        public:

        Tregion(){};

        //gets the regions id
        string getId() {return this->id;};

        //moves on to the next agent and gets it
        Ped::Tagent *getNext();

        //Initialises the iterator and gets the start
        Ped::Tagent *getStart();

        //move current agent to another region
        bool moveAgentExternally(Tregion *region, std::pair<int, int> position);

        // Move within the same region.
        bool moveAgentInternally(std::pair<int, int> position);


        bool isAvailable(int x, int y);

        //finds out whether position is in this region
        //bool isInRegion(int x, int y);

        //gets the lock
        void getLock();

        //unlocks the lock
        void unlockLock();

        //append
        void append( Ped::Tagent *agent);


        private:

        Ped::Tagent * pop();
        string get_uuid();

        //int greaterRegion;
        //int xStart;
        //int xEnd;
        string id = get_uuid();


        Ped::Tagent *current = NULL;
        Ped::Tagent **previous = NULL;

        std::mutex agentsLock;
        Ped::Tagent *startAgent = NULL;
        Ped::Tagent *endAgent = NULL;

    };
}
