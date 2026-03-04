//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_model.h"
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <omp.h>
#include <emmintrin.h>
#include <immintrin.h>
#include <thread>
#include <vector>

#ifndef NOCDUA
#include "cuda_testkernel.h"
#endif

#include <stdlib.h>
#include <unistd.h>

void Ped::Model::setup(std::vector<Ped::Tagent *> agentsInScenario,
                       std::vector<Twaypoint *> destinationsInScenario,
                       void **positionArrays,
                       IMPLEMENTATION implementation)
{
#ifndef NOCUDA
    // Convenience test: does CUDA work on this machine?
    cuda_test();
#else
    std::cout << "Not compiled for CUDA" << std::endl;
#endif

    agentX = (int *)positionArrays[0];
    agentY = (int *)positionArrays[1];
    destinationX = (double *)positionArrays[2];
    destinationY = (double *)positionArrays[3];
    desiredX = (int *)positionArrays[4];
    desiredY = (int *)positionArrays[5];
    free(positionArrays);

    // Set
    agents = std::vector<Ped::Tagent *>(agentsInScenario.begin(),
                                        agentsInScenario.end());

    positionTracker.setup(agents);

    // Set up destinations
    destinations = std::vector<Ped::Twaypoint *>(destinationsInScenario.begin(),
                                                 destinationsInScenario.end());
    // Regions to manage, each region gets assigned to one thread.
    int noOfRegions = 120 / REGION_HEIGHT; // total height is 120, need total height/height per region regions
    regions = std::vector<
        std::map<int, std::pair<Ped::Tregion *, bool>>>(noOfRegions);
    for (int i = 0; i < regions.size(); i++)
    {
        Ped::Tregion *region = new Tregion(i << 8, 0, 159);
        std::map<int, std::pair<Ped::Tregion *, bool>> regionMap;
        regionMap[0] = std::make_pair(region, false);
        regionMap[159] = std::make_pair(region, true);
        regions[i] = regionMap;
    }
    for (int i = 0; i < agents.size(); i++)
    {
        Tregion *region = calculateRegion(agents[i]->getX(), agents[i]->getY());
        region->append(agents[i]);
    }

    // Used for cuda.
    agentsDesiredX = (int *)malloc(agents.size()*sizeof(int));
    agentsDesiredY = (int *)malloc(agents.size()*sizeof(int));

    // Sets the chosen implemenation. Standard in the given code is SEQ
    this->implementation = implementation;

    // Set up heatmap (relevant for Assignment 4)
    // Uncomment and comment the other to switch between seq and cuda versions.
    // setupHeatmapSeq();
    setupHeatmapCuda();
}

void Ped::Model::freePosArrs()
{
    free(agentX);
    free(agentY);
    free(destinationX);
    free(destinationY);
    free(desiredX);
    free(desiredY);
}

Ped::Tregion *Ped::Model::calculateRegion(int x, int y)
{
    // assuming horizontal stripes
    int rem = y % REGION_HEIGHT;
    if (rem < 0)
    {
        rem += REGION_HEIGHT;
    }
    // printf("%d\n", y);
    // printf("%d\n", rem);
    int index = ((y - rem) / REGION_HEIGHT); // Tog bort -1, tror att det ska vara så här
    // printf("%d\n", index);
    auto regionMap = regions[index];
    Ped::Tregion *region;
    while (true)
    {
        auto it = regionMap.find(x);
        if (it != regionMap.end())
        {
            return it->second.first;
        }
        x++;
        if (x >= 160)
        {
            throw std::runtime_error(
                "calculateRegion: No region found for position(" //
                + std::to_string(x)                              //
                + ", " + std::to_string(y)                       //
                + ")");                                          //
        }
    }
}

void Ped::Model::sequentialTick()
{
    for (int i = 0; i < agents.size(); i++)
    {
        Ped::Tagent *agent = agents[i];
        agent->computeNextDesiredPosition();

        agent->setX(agent->getDesiredX());
        agent->setY(agent->getDesiredY());
    }
}

void Ped::Model::collisionSequentialTick()
{
    for (int i = 0; i < agents.size(); i++)
    {
        Ped::Tagent *agent = agents[i];
        agent->computeNextDesiredPosition();

        move(agent);
    }
}

std::vector<Ped::Tregion *> Ped::Model::collectRegions()
{
    std::vector<Ped::Tregion *> allRegions;
    for (auto &regionMap : regions)
    {
        for (auto &regionPair : regionMap)
        {
            auto payload = regionPair.second;
            if (!payload.second)
            {
                continue; // skip duplicate pointer to the same region
            }
            allRegions.push_back(payload.first);
        }
    }
    return allRegions;
}

std::vector<std::pair<int, int>> calcAlternativePositions(Ped::Tagent *agent)
{

    // Compute where the agent wants to go
    // and the alternative positions to try if the desired position is taken
    // where one of the alternative positions is a tangential step
    // this is to prevent deadlocks where agents are blocking each other
    std::vector<std::pair<int, int>> alternativePositions;
    std::pair<int, int> pDesired(agent->getDesiredX(), agent->getDesiredY());
    alternativePositions.push_back(pDesired); // add desired position as an alternative

    int diffX = pDesired.first - agent->getX();
    int diffY = pDesired.second - agent->getY();
    std::pair<int, int> p1, p2, p3;
    if (diffX == 0 || diffY == 0)
    {
        // Agent wants to walk straight to North, South, West or East
        p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
        p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
        p3 = std::make_pair(agent->getX() + diffY, agent->getY() + diffX);
    }
    else
    {
        // Agent wants to walk diagonally
        p1 = std::make_pair(pDesired.first, agent->getY());
        p2 = std::make_pair(agent->getX(), pDesired.second);
        p3 = std::make_pair(pDesired.first, agent->getY() - diffY);
    }

    alternativePositions.push_back(p1);
    alternativePositions.push_back(p2);
    alternativePositions.push_back(p3);

    return alternativePositions;
}

void Ped::Model::dynamicResizeRegions()
{
#pragma omp parallel for schedule(dynamic) shared(regions)
    for (int i = 0; i < regions.size(); i++)
    {
        auto currentMap = &regions[i];
        auto itr = currentMap->begin();
        while (itr != currentMap->end())
        {
            if (!itr->second.second)
            {
                itr++;
            }
            Ped::Tregion *region = itr->second.first;

            if (region->getAmountOfAgents() > REGION_HEIGHT * 6)
            {
                Ped::Tregion *newRegion = region->splitRegion();
                int border = newRegion->getXStart();
                currentMap->insert(std::make_pair(border, std::make_pair(newRegion, false)));
                currentMap->insert(std::make_pair(border - 1, std::make_pair(region, true)));
                itr->second = make_pair(newRegion, true);
                itr++;
                itr++;
            }
            else if (region->getAmountOfAgents() < REGION_HEIGHT)
            {
                auto nextItr = std::next(itr);
                if (nextItr == currentMap->end())
                {
                    break;
                }
                Ped::Tregion *nextRegion = nextItr->second.first;
                if (nextRegion->getAmountOfAgents() < REGION_HEIGHT)
                {

                    region->mergeRegion(nextRegion);

                    currentMap->erase(nextItr);
                    itr = currentMap->erase(itr);
                    itr->second = make_pair(region, true);
                    itr++;
                }
            }

            itr++;
        }
    }
}

void Ped::Model::parrallelCMove()
{
    std::vector<Ped::Tregion *> allRegions = this->collectRegions();
#pragma omp parallel for schedule(dynamic) shared(regions)
    for (int i = 0; i < allRegions.size(); i++)
    {

        Ped::Tregion *region = allRegions[i];
        Ped::Tagent *agent = region->getStart();
        while (agent != NULL)
        {

            auto altPos = calcAlternativePositions(agent);

            std::pair<int, int> currentPos(agent->getX(), agent->getY());
            bool hasPopped = false;
            for (int i = 0; i < altPos.size(); i++)
            {
                std::pair<int, int> choosenPos = altPos[i];

                if (!positionTracker.tryMoveAgent(currentPos, choosenPos))
                {
                    continue; // desired position is taken, try next alternative position
                }

                agent->setX(choosenPos.first);
                agent->setY(choosenPos.second);
                agent->setHasMoved(true);

                Tregion *newRegion = calculateRegion(choosenPos.first, choosenPos.second);

                if (region->getId() != newRegion->getId())
                {
                    hasPopped = true;
                    agent = region->moveCurrentToAnotherRegion(newRegion);
                }
                break;
            }
            if (!hasPopped)
            {
                agent = region->getNext();
            }
        };
    }
}

void Ped::Model::collisionOMPTick()
{
    this->ticks++;

    // Reset hasMoved and computes the desired position for all agents in parallel
#pragma omp parallel for schedule(static)
    for (int i = 0; i < agents.size(); i++)
    {
        Ped::Tagent *agent = agents[i];
        agent->setHasMoved(false);
        agent->computeNextDesiredPosition();
    }
    updateHeatmapSeq();

    if (this->ticks % 400 == 40)
    {
        dynamicResizeRegions();
    }

    this->parrallelCMove();
}

void Ped::Model::collisionOMPHMTick()
{
    this->ticks++;

    // Reset hasMoved and computes the desired position for all agents in parallel
#pragma omp parallel for schedule(static)
    for (int i = 0; i < agents.size(); i++)
    {
        Ped::Tagent *agent = agents[i];
        agent->setHasMoved(false);
        agent->computeNextDesiredPosition();
        agentsDesiredX[i] = agent->getDesiredX();
        agentsDesiredY[i] = agent->getDesiredY();
    }

    updateHeatmapCuda(agentsDesiredX, agentsDesiredY, agents.size());

    if (this->ticks % 400 == 40)
    {
        dynamicResizeRegions();
    }

    this->parrallelCMove();
}

void Ped::Model::collisionOMPSIMDTick()
{
    this->ticks++;

    // Reset hasMoved and computes the desired position for all agents in parallel
#pragma omp parallel for schedule(static)
    for (int i = 0; i < agents.size(); i++)
    {
        Ped::Tagent *agent = agents[i];
        agent->setHasMoved(false);
    }
    this->simdCompNextDesiredPos();

    if (this->ticks % 400 == 40)
    {
        dynamicResizeRegions();
    }

    this->parrallelCMove();
}

void Ped::Model::simdCompNextDesiredPos()
{

    for (int i = 0; i < agents.size(); i++)
    {
        Ped::Tagent *agent = agents[i];
        agent->setNextDestination();
    }
    for (int i = 0; i < agents.size(); i += 4)
    {

        __m256d destX = _mm256_load_pd(&destinationX[i]);
        __m256d destY = _mm256_load_pd(&destinationY[i]);
        __m128i posX = _mm_loadu_si128((__m128i *)&agentX[i]);
        __m128i posY = _mm_loadu_si128((__m128i *)&agentY[i]);

        __m256d posXD = _mm256_cvtepi32_pd(posX);
        __m256d posYD = _mm256_cvtepi32_pd(posY);

        __m256d diffX = _mm256_sub_pd(destX, posXD);
        __m256d diffY = _mm256_sub_pd(destY, posYD);

        __m256d squaredDiffX = _mm256_mul_pd(diffX, diffX);
        __m256d squaredSum = _mm256_fmadd_pd(diffY, diffY, squaredDiffX);
        __m256d length = _mm256_sqrt_pd(squaredSum);

        __m256d zero = _mm256_set1_pd(0.0);

        __m256d zeroLenMask = _mm256_cmp_pd(length, zero, _CMP_NEQ_OQ);

        __m256d normDiffX = _mm256_div_pd(diffX, length);
        __m256d normDiffY = _mm256_div_pd(diffY, length);

        __m256d normDiffXMasked = _mm256_and_pd(normDiffX, zeroLenMask);
        __m256d normDiffYMasked = _mm256_and_pd(normDiffY, zeroLenMask);

        __m256d desiredXD = _mm256_add_pd(posXD, normDiffXMasked);
        __m256d desiredYD = _mm256_add_pd(posYD, normDiffYMasked);
        __m256d roundedX = _mm256_round_pd(desiredXD, (_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
        __m256d roundedY = _mm256_round_pd(desiredYD, (_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));

        __m128i desireX = _mm256_cvtpd_epi32(roundedX);
        __m128i desireY = _mm256_cvtpd_epi32(roundedY);

        _mm_storeu_si128((__m128i *)&desiredX[i], desireX);
        _mm_storeu_si128((__m128i *)&desiredY[i], desireY);
    }
}

void Ped::Model::simdTick()
{

    this->simdCompNextDesiredPos();

    for (int i = 0; i < agents.size(); i += 4)
    {
        __m128i desireX = _mm_loadu_si128((__m128i *)&desiredX[i]);
        __m128i desireY = _mm_loadu_si128((__m128i *)&desiredY[i]);
        _mm_storeu_si128((__m128i *)&agentX[i], desireX);
        _mm_storeu_si128((__m128i *)&agentY[i], desireY);
    }
}

void Ped::Model::ompTick()
{
#pragma omp parallel for num_threads(8)
    for (int i = 0; i < agents.size(); i++)
    {
        Ped::Tagent *agent = agents[i];
        agent->computeNextDesiredPosition();
        agent->setX(agent->getDesiredX());
        agent->setY(agent->getDesiredY());
    }
}

void cppTickInternal(std::vector<Ped::Tagent *> agents, int threadID,
                     int iterationCount)
{
    for (int i = 0; i < agents.size() / iterationCount; i++)
    {
        Ped::Tagent *agent =
            (agents)[threadID * (agents.size() / iterationCount) + i];
        agent->computeNextDesiredPosition();
        agent->setX(agent->getDesiredX());
        agent->setY(agent->getDesiredY());
    }
}

void Ped::Model::cppTick()
{
    int newThreadsCount = 3;
    int iterationCount = newThreadsCount + 1;
    std::thread threads[newThreadsCount];

    for (int threadID = 0; threadID < newThreadsCount; threadID++)
    {
        threads[threadID] =
            std::thread(cppTickInternal, agents, threadID, iterationCount);
    }
    // Use main thread for remainder work that thread dispatch didn't do.
    for (int i = 0; i < agents.size() - ((agents.size() / iterationCount) *
                                         (newThreadsCount));
         i++)
    {
        Ped::Tagent *agent =
            agents[(newThreadsCount) * (agents.size() / iterationCount) + i];
        agent->computeNextDesiredPosition();
        agent->setX(agent->getDesiredX());
        agent->setY(agent->getDesiredY());
    }
    for (int i = 0; i < newThreadsCount; i++)
    {
        threads[i].join();
    }
}

void Ped::Model::tick()
{

    if (this->implementation == OMP)
    {
        ompTick();
    }
    else if (this->implementation == PTHREAD)
    {
        cppTick();
    }
    else if (this->implementation == VECTOR)
    {
        simdTick();
    }
    else if (this->implementation == COLLISION_SEQ)
    {
        collisionSequentialTick();
    }
    else if (this->implementation == COLLISION_OMP)
    {
        //collisionOMPTick();
        collisionOMPHMTick(); // Temp for cuda, assignement 4
    }
    else if (this->implementation == COLLISION_OMP_SIMD)
    {
        collisionOMPSIMDTick();
    }
    else
    {
        sequentialTick();
    }
}

////////////
/// Everything below here relevant for Assignment 3.
/// Don't use this for Assignment 1!
///////////////////////////////////////////////

// Moves the agent to the next desired position. If already taken, it will
// be moved to a location close to it.
void Ped::Model::move(Ped::Tagent *agent)
{
    // Search for neighboring agents
    set<const Ped::Tagent *> neighbors =
        getNeighbors(agent->getX(), agent->getY(), 2);

    // Retrieve their positions
    std::vector<std::pair<int, int>> takenPositions;
    for (std::set<const Ped::Tagent *>::iterator neighborIt = neighbors.begin();
         neighborIt != neighbors.end(); ++neighborIt)
    {
        std::pair<int, int> position((*neighborIt)->getX(), (*neighborIt)->getY());
        takenPositions.push_back(position);
    }

    // Compute the three alternative positions that would bring the agent
    // closer to his desiredPosition, starting with the desiredPosition itself
    std::vector<std::pair<int, int>> prioritizedAlternatives;
    std::pair<int, int> pDesired(agent->getDesiredX(), agent->getDesiredY());
    prioritizedAlternatives.push_back(pDesired);

    int diffX = pDesired.first - agent->getX();
    int diffY = pDesired.second - agent->getY();
    std::pair<int, int> p1, p2;
    if (diffX == 0 || diffY == 0)
    {
        // Agent wants to walk straight to North, South, West or East
        p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
        p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
    }
    else
    {
        // Agent wants to walk diagonally
        p1 = std::make_pair(pDesired.first, agent->getY());
        p2 = std::make_pair(agent->getX(), pDesired.second);
    }
    prioritizedAlternatives.push_back(p1);
    prioritizedAlternatives.push_back(p2);

    // Find the first empty alternative position
    for (std::vector<pair<int, int>>::iterator it =
             prioritizedAlternatives.begin();
         it != prioritizedAlternatives.end(); ++it)
    {

        // If the current position is not yet taken by any neighbor
        if (std::find(takenPositions.begin(), takenPositions.end(), *it) ==
            takenPositions.end())
        {

            // Set the agent's position
            agent->setX((*it).first);
            agent->setY((*it).second);

            break;
        }
    }
}

/// Returns the list of neighbors within dist of the point x/y. This
/// can be the position of an agent, but it is not limited to this.
/// \date    2012-01-29
/// \return  The list of neighbors
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents
/// (search field is a square in the current implementation)
set<const Ped::Tagent *> Ped::Model::getNeighbors(int x, int y,
                                                  int dist) const
{

    // create the output list
    // ( It would be better to include only the agents close by, but this
    // programmer is lazy.)
    return set<const Ped::Tagent *>(agents.begin(), agents.end());
}

void Ped::Model::cleanup()
{
    // Nothing to do here right now.
}

Ped::Model::~Model()
{
    std::for_each(agents.begin(), agents.end(),
                  [](Ped::Tagent *agent)
                  { delete agent; });
    std::for_each(destinations.begin(), destinations.end(),
                  [](Ped::Twaypoint *destination)
                  { delete destination; });
}
