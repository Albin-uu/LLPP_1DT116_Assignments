#include "ped_region.h"
#include "ped_agent.h"
#include <random>
#include <cstddef>
#include <queue>
#include <iostream>

Ped::Tagent *Ped::Tregion::getStart()
{
    previous = &startAgent;
    current = this->startAgent;
    return current;
}

Ped::Tagent *Ped::Tregion::getNext()
{
    do
    {
        this->previous = this->current->getNextAgentField();
        this->current = this->current->getNextAgent();
    } while (this->current != NULL && this->current->getHasMoved());

    return this->current;
}

void Ped::Tregion::append(Ped::Tagent *agent)
{
    this->amountOfAgents++;
    agent->setNextAgent(NULL);
    std::atomic<Ped::Tagent *> *prevEnd = std::atomic_exchange(&this->endField, agent->getNextAgentField());
    prevEnd->store(agent);
}

Ped::Tagent *Ped::Tregion::pop()
{
    this->amountOfAgents--;

    Ped::Tagent *popped = current;
    std::atomic<Ped::Tagent *> *nextField = popped->getNextAgentField();

    if (!std::atomic_compare_exchange_strong(&this->endField, &nextField, previous))
    {
        // branches if current is not the end of the list
        nextField = popped->getNextAgentField();

        // if the next field is null, the list just got appended to,
        // so we wait until the append has fully updated the link
        while (nextField->load() == NULL)
            ;
    }

    Ped::Tagent *next = popped->getNextAgent();
    std::atomic_compare_exchange_weak(previous, &current, next);
    current = next;

    return popped;
}

int Ped::Tregion::findMedianX()
{
    priority_queue<int> small, large;
    Tagent *agent = this->getStart();
    while (agent != NULL)
    {
        small.push(agent->getX());
        large.push(-small.top());
        small.pop();
        if (small.size() < large.size())
        {
            small.push(-large.top());
            large.pop();
        }
        agent = agent->getNextAgent();
    }

    return small.top();
}

Ped::Tregion *Ped::Tregion::splitRegion()
{

    int median = this->findMedianX();

    Tregion *newRegion = new Tregion((this->id & REGION_ID_Y) | median, median, this->xEnd);
    Tagent *agent = this->getStart();
    while (agent != NULL)
    {

        if (agent->getX() >= median)
        {
            agent = this->moveCurrentToAnotherRegion(newRegion);
        }
        else
        {
            agent = this->getNext();
        }
    }

    this->xEnd = median - 1;

    return newRegion;
}

void Ped::Tregion::mergeRegion(Ped::Tregion *otherRegion)
{
    Tagent *agent = otherRegion->getStart();
    while (agent != NULL)
    {
        agent = otherRegion->moveCurrentToAnotherRegion(this);
    }
    this->xEnd = otherRegion->xEnd;
    delete otherRegion;
}

Ped::Tagent *Ped::Tregion::moveCurrentToAnotherRegion(Tregion *otherRegion)
{
    Ped::Tagent *popped = this->pop();
    otherRegion->append(popped);
    if (current != NULL && current->getHasMoved())
    {
        return this->getNext();
    }
    return current;
}

string Ped::Tregion::get_uuid()
{
    static random_device dev;
    static mt19937 rng(dev());

    uniform_int_distribution<int> dist(0, 15);

    const char *v = "0123456789abcdef";
    const bool dash[] = {0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0};

    string res;
    for (int i = 0; i < 16; i++)
    {
        if (dash[i])
            res += "-";
        res += v[dist(rng)];
        res += v[dist(rng)];
    }
    res = res + "\0";
    return res;
}
