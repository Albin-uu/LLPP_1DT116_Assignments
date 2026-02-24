#include "ped_region.h"
#include "ped_agent.h"
#include <random>
#include <cstddef>

void Ped::Tregion::getLock() {
    agentsLock.lock();
}

void Ped::Tregion::unlockLock() {
    agentsLock.unlock();
}

Ped::Tagent *Ped::Tregion::getStart() {
    previous = &startAgent;
    current = this->startAgent;
    return current;
}


Ped::Tagent *Ped::Tregion::getNext() {
    if (this->current == NULL) {
        return NULL;
    }
    getLock();
    do {
        Tagent *oldCurrent = this->current;
        this->current = this->current->getNextAgent();
        this->previous = oldCurrent->getNextAgentField();
        if (this->current == NULL) {
            unlockLock();
            return NULL;
        }
    } while(this->current->getHasMoved());

    unlockLock();
    return this->current;
}


//only works if the getStrart has been called need fix
void Ped::Tregion::append( Ped::Tagent *agent) {
    if (this->startAgent == NULL) {
        this->startAgent = agent;
    }

    if(endAgent != NULL) {
    this->endAgent->setNextAgent(agent);
    }
    this->endAgent = agent;
    agent->setNextAgent(NULL);
}

Ped::Tagent * Ped::Tregion::pop(){
    Ped::Tagent *popped = current;
    *previous=popped->getNextAgent();
    current=popped->getNextAgent();
    return popped;

}

bool Ped::Tregion::moveAgentExternally(Tregion *region, std::pair<int, int> position) {
    if (current == NULL) {return false;}
    region->getLock();
    int x = position.first;
    int y = position.second;
    if (!region->isAvailable(x, y)) {
        region->unlockLock();
        return false;
    }
    current->setX(x);
    current->setY(y);
    current->setHasMoved(true);
    region->append(this->pop());
    region->unlockLock();
    return true;
}

bool Ped::Tregion::moveAgentInternally(std::pair<int, int> position) {
    if (current == NULL) {return false;}
    this->getLock();
    int x = position.first;
    int y = position.second;
    if (!this->isAvailable(x, y)) {
        this->unlockLock();
        return false;
    }
    current->setX(x);
    current->setY(y);
    current->setHasMoved(true);
    this->unlockLock();
    return false;
}

bool Ped::Tregion::isAvailable(int x, int y) {
    for (Ped::Tagent *agent = startAgent; agent != NULL; agent = agent->getNextAgent()) {
        if (agent->getX() == x && agent->getY() == y) {
            return false;
        }
    }
    return true;
}

string Ped::Tregion::get_uuid() {
    static random_device dev;
    static mt19937 rng(dev());

    uniform_int_distribution<int> dist(0, 15);

    const char *v = "0123456789abcdef";
    const bool dash[] = { 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0 };

    string res;
    for (int i = 0; i < 16; i++) {
        if (dash[i]) res += "-";
        res += v[dist(rng)];
        res += v[dist(rng)];
    }
    res = res + "\0";
    return res;
}
