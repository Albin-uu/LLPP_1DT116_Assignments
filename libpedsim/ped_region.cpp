#include "ped_region.h"
#include "ped_agent.h"
#include <random>
#include <cstddef>

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
    std::atomic<Ped::Tagent *> *prevEnd = std::atomic_exchange(&this->endField, agent->getNextAgentField());
    prevEnd->store(agent);
}

Ped::Tagent *Ped::Tregion::pop()
{
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

void Ped::Tregion::moveCurrentToAnotherRegion(Tregion *region)
{
    Ped::Tagent *popped = this->pop();
    region->append(popped);
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
