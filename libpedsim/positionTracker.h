#include <stdlib.h>
#include <utility>
#include <atomic>
#include <vector>
#include "ped_agent.h"

namespace Ped
{
    class Tpositions
    {
    public:
        // Initializes the position tracker with the positions of the agents
        void setup(std::vector<Ped::Tagent *> agents);

        // Moves the agent to the desired position if possible, and returns whether the move was successful
        bool tryMoveAgent(std::pair<int, int> currentPos, std::pair<int, int> desiredPos);

    private:
        // Places the agent at the given position, and returns whether the placement was successful
        bool placeAgent(std::pair<int, int> pos);

        std::pair<int, int> posToIndex(std::pair<int, int> pos);

        std::atomic_ulong *bitmasks;
    };
}