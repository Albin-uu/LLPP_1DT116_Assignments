#include "ped_region.h"
#include "ped_agent.h"
#include <random>
#include <cstddef>

void Ped::Tregion::getLock()
{
    agentsLock.lock();
}

void Ped::Tregion::unlockLock()
{
    agentsLock.unlock();
}

Ped::Tagent *Ped::Tregion::getStart()
{
    previous = &startAgent;
    current = this->startAgent;
    return current;
}

Ped::Tagent *Ped::Tregion::getNext()
{
    if (this->current == NULL)
    {
        return NULL;
    }
    do
    {
        Tagent *oldCurrent = this->current;
        this->current = this->current->getNextAgent();
        this->previous = oldCurrent->getNextAgentField();
        if (this->current == NULL)
        {
            return NULL;
        }
    } while (this->current->getHasMoved());

    return this->current;
}

void Ped::Tregion::append(Ped::Tagent *agent)
{
    agent->setNextAgent(NULL);
    Ped::Tagent **prevEnd = std::atomic_exchange(&this->endField, agent->getNextAgentField());
    *prevEnd = agent;
}

Ped::Tagent *Ped::Tregion::pop()
{
    Ped::Tagent *popped = current;
    Ped::Tagent **nextField = popped->getNextAgentField();
    Ped::Tagent *next = popped->getNextAgent();
    *previous = next;
    current = next;
    std::atomic_compare_exchange_weak(&this->endField, &nextField, previous);

    return popped;
}

void Ped::Tregion::moveCurrentToAnotherRegion(Tregion *region)
{
    this->getLock();
    Ped::Tagent *popped = this->pop();
    this->unlockLock();
    region->getLock();
    region->append(popped);
    region->unlockLock();
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
