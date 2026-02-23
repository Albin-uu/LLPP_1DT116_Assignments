

#include "ped_agent.h"
#include <mutex>
namespace Ped
{
    class Tregion
    {
        public:

        //gets the regions id
        int getId() {return id;};

        //moves on to the next agent and gets it
        Ped::Tagent *getNext();

        //Initialises the iterator and gets the start
        Ped::Tagent *getStart();

        //move current agent to another region
        bool moveAgentExternally(Tregion *region);

        // Move within the same region.
        bool moveAgentInternally();


        bool isAvaliable(int x, int y);

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

        //int greaterRegion;
        //int xStart;
        //int xEnd;
        int id;

        Ped::Tagent *current;
        Ped::Tagent **previous;

        std::mutex agentsLock;
        Ped::Tagent *startAgent;
        Ped::Tagent *endAgent;

    };
}
