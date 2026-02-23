#include "ped_region.h"
#include "ped_agent.h"
#include <cstddef>

void Ped::Tregion::getLock() {
    agentsLock.lock();
}

void Ped::Tregion::unlockLock() {
    agentsLock.unlock();
}

//needs fix
Ped::Tagent *Ped::Tregion::getStart() {
    previous = &startAgent;
    current = startAgent;
    return startAgent;
}


Ped::Tagent *Ped::Tregion::getNext() {
    getLock();
    do {
        if (current == NULL) {
            unlockLock();
            return NULL;
        }
        current = current->getNextAgent();
        previous = current->getNextAgentField();
    } while(current->getHasMoved());

    unlockLock();
    return current;
}


//only works if the getStrart has been called need fix
void Ped::Tregion::append( Ped::Tagent *agent) {
    endAgent->setNextAgent(agent);
    endAgent = agent;
    agent->setNextAgent(NULL);
}

Ped::Tagent * Ped::Tregion::pop(){
    Ped::Tagent *popped = current;
    *previous=popped->getNextAgent();
    current=popped->getNextAgent();
    return popped;

}

bool Ped::Tregion::moveAgentExternally(Tregion *region) {
    region->getLock();
    int x = current->getDesiredX();
    int y = current->getDesiredY();
    if (!region->isAvaliable(x, y)) {
        region->unlockLock();
        return false;
    }
    current->setX(x);
    current->setY(y);
    current->setHasMoved(true);
    region->append(pop());
    region->unlockLock();
    return true;
}

bool Ped::Tregion::moveAgentInternally() {
    // TODO
    return false;
}
