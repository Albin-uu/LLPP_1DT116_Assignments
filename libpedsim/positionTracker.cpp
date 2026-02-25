#include "positionTracker.h"
#include <assert.h>
#include <iostream>

// bits in a bitmask
#define BIMASK_SIZE sizeof(unsigned long) * 8

std::pair<int, int> Ped::Tpositions::posToIndex(std::pair<int, int> pos)
{
    int raw = (pos.first + pos.second * 160);
    // out.fisrt: index of bitmask in array
    // out.second: index of bit in bitmask
    return std::make_pair(raw / BIMASK_SIZE, raw % BIMASK_SIZE);
}

bool Ped::Tpositions::placeAgent(std::pair<int, int> pos)
{
    std::pair<int, int> index = posToIndex(pos);
    u_long mask = ((u_long)1 << index.second);
    return (std::atomic_fetch_or(bitmasks + index.first, mask) & mask) == 0;
}

bool Ped::Tpositions::moveAgent(std::pair<int, int> currentPos, std::pair<int, int> desiredPos)
{
    std::pair<int, int> currentIndex = posToIndex(currentPos);

    u_long currentMask = ((u_long)1 << currentIndex.second);

    if (placeAgent(desiredPos)) // If desired position is available, move agent there
    {
        // Mark current position as available
        std::atomic_fetch_xor(bitmasks + currentIndex.first, currentMask);
        return true;
    }
    else
    {
        return false;
    }
}

void Ped::Tpositions::setup(std::vector<Ped::Tagent *> agents)
{
    int size = (160 * 120) / BIMASK_SIZE;
    bitmasks = (std::atomic_ulong *)malloc(size * sizeof(std::atomic_ulong));
    for (int i = 0; i < size; i++)
    {
        atomic_init(bitmasks + i, 0);
    }
    for (Ped::Tagent *agent : agents)
    {
        assert(placeAgent(std::make_pair(agent->getX(), agent->getY())));
    }
}