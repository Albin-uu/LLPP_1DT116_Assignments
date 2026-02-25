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
        void setup(std::vector<Ped::Tagent *> agents);

        bool moveAgent(std::pair<int, int> currentPos, std::pair<int, int> desiredPos);

    private:
        bool placeAgent(std::pair<int, int> pos);

        std::pair<int, int> posToIndex(std::pair<int, int> pos);

        std::atomic_ulong *bitmasks;
    };
}