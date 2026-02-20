//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_model.h"
#include "ped_waypoint.h"
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <omp.h>
#include <stack>
#include <emmintrin.h>
#include <immintrin.h>
#include <thread>

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
    // Set up destinations
    destinations = std::vector<Ped::Twaypoint *>(destinationsInScenario.begin(),
                                                 destinationsInScenario.end());

    // Sets the chosen implemenation. Standard in the given code is SEQ
    this->implementation = implementation;

    // Set up heatmap (relevant for Assignment 4)
    setupHeatmapSeq();
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

void Ped::Model::sequentialTick()
{
    for (int i = 0; i < agents.size(); i++)
    {
        Ped::Tagent *agent = agents[i];
        agent->computeNextDesiredPosition();

        agent->setX(agent->getDesiredX());
        agent->setY(agent->getDesiredY());
    }
    return;
}

void Ped::Model::collisionSequentialTick()
{
    for (int i = 0; i < agents.size(); i++)
    {
        Ped::Tagent *agent = agents[i];
        agent->computeNextDesiredPosition();

        move(agent);
    }
    return;
}

void Ped::Model::collisionParallelTick()
{
    for (int i = 0; i < agents.size(); i++)
    {
        Ped::Tagent *agent = agents[i];
        agent->computeNextDesiredPosition();

        // TODO
        move(agent);
    }
    return;
}

void Ped::Model::simdTick()
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

        __m256d normDiffX = _mm256_div_pd(diffX, length);
        __m256d normDiffY = _mm256_div_pd(diffY, length);

        __m256d desiredXD = _mm256_add_pd(posXD, normDiffX);
        __m256d desiredYD = _mm256_add_pd(posYD, normDiffY);

        __m256d roundedX = _mm256_round_pd(desiredXD, (_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
        __m256d roundedY = _mm256_round_pd(desiredYD, (_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));

        __m128i desireX = _mm256_cvtpd_epi32(roundedX);
        __m128i desireY = _mm256_cvtpd_epi32(roundedY);

        _mm_storeu_si128((__m128i *)&desiredX[i], desireX);
        _mm_storeu_si128((__m128i *)&desiredY[i], desireY);
        _mm_storeu_si128((__m128i *)&agentX[i], desireX);
        _mm_storeu_si128((__m128i *)&agentY[i], desireY);
    }
    return;
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
    return;
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
    return;
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
    return;
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
    else if (this->implementation == COLLISION_PARA)
    {
        collisionParallelTick();
    }
    else
    {
        sequentialTick();
    }

    return;
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
