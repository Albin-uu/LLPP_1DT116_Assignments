//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
// Adapted for Low Level Parallel Programming 2017.
// Modified in 2025 to remove QT's XML parser and used TinyXML2 instead.

#include "ParseScenario.h"
#include <string>
#include <iostream>
#include <set>

#include <stdlib.h>

// Comparator used to identify if two agents differ in their position
bool positionComparator(Ped::Tagent *a, Ped::Tagent *b)
{
    // True if positions of agents differ
    return (a->getX() < b->getX()) || ((a->getX() == b->getX()) && (a->getY() < b->getY()));
}

// Reads in the configuration file, given the filename
ParseScenario::ParseScenario(std::string filename, bool verbose)
{

    XMLError ret = doc.LoadFile(filename.c_str());
    if (ret != XML_SUCCESS)
    {
        // std::cout << "Error reading the scenario configuration file: " << ret << std::endl;
        fprintf(stderr, "Error reading the scenario configuration file for filename %s: ", filename.c_str());
        perror(NULL);
        exit(1);
        return;
    }

    // Get the root element (welcome)
    XMLElement *root = doc.FirstChildElement("welcome");
    if (!root)
    {
        std::cerr << "Error: Missing <welcome> element in the XML file!" << std::endl;
        exit(1);
        return;
    }

    // Parse waypoints
    if (verbose)
        std::cout << "Waypoints:" << std::endl;
    for (XMLElement *waypoint = root->FirstChildElement("waypoint"); waypoint; waypoint = waypoint->NextSiblingElement("waypoint"))
    {
        std::string id = waypoint->Attribute("id");
        double x = waypoint->DoubleAttribute("x");
        double y = waypoint->DoubleAttribute("y");
        double r = waypoint->DoubleAttribute("r");

        if (verbose)
            std::cout << "  ID: " << id << ", x: " << x << ", y: " << y << ", r: " << r << std::endl;

        Ped::Twaypoint *w = new Ped::Twaypoint(x, y, r);
        waypoints[id] = w;
    }

    // Parse agents
    if (verbose)
        std::cout << "\nAgents:" << std::endl;
    for (XMLElement *agent = root->FirstChildElement("agent"); agent; agent = agent->NextSiblingElement("agent"))
    {
        double x = agent->DoubleAttribute("x");
        double y = agent->DoubleAttribute("y");
        int n = agent->IntAttribute("n");
        double dx = agent->DoubleAttribute("dx");
        double dy = agent->DoubleAttribute("dy");

        if (verbose)
        {
            std::cout << "  Agent: x: " << x << ", y: " << y << ", n: " << n
                      << ", dx: " << dx << ", dy: " << dy << std::endl;
        }

        tempAgents.clear();
        positionArrays = (void **)malloc(6 * sizeof(void *));
        int *agentX = (int *)malloc(n * sizeof(int));
        int *agentY = (int *)malloc(n * sizeof(int));
        double *destinationX = (double *)malloc(n * sizeof(double));
        double *destinationY = (double *)malloc(n * sizeof(double));
        int *desiredX = (int *)malloc(n * sizeof(int));
        int *desiredY = (int *)malloc(n * sizeof(int));
        positionArrays[0] = agentX;
        positionArrays[1] = agentY;
        positionArrays[2] = destinationX;
        positionArrays[3] = destinationY;
        positionArrays[4] = desiredX;
        positionArrays[5] = desiredY;
        set<long> usedPos{};
        for (int i = 0; i < n; ++i)
        {
            int tempX;
            int tempY;
            long setIndex;
            do
            {
                tempX = x + rand() / (RAND_MAX / dx) - dx / 2;
                tempY = y + rand() / (RAND_MAX / dy) - dy / 2;
                setIndex = (long)tempX << 32 + (long)tempY;

            } while (usedPos.find(setIndex) != usedPos.end());
            usedPos.insert(setIndex);
            agentX[i] = tempX;
            agentY[i] = tempY;

            Ped::Tagent *a = new Ped::Tagent(positionArrays, i);
            tempAgents.push_back(a);
        }

        agents.insert(agents.end(), tempAgents.begin(), tempAgents.end());
    }
    tempAgents.clear();

    // Hack! Do not allow agents to be on the same position. Remove duplicates from scenario and free the memory.
    bool (*fn_pt)(Ped::Tagent *, Ped::Tagent *) = positionComparator;
    std::set<Ped::Tagent *, bool (*)(Ped::Tagent *, Ped::Tagent *)> agentsWithUniquePosition(fn_pt);
    int duplicates = 0;
    for (auto agent : agents)
    {
        if (agentsWithUniquePosition.find(agent) == agentsWithUniquePosition.end())
        {
            agentsWithUniquePosition.insert(agent);
        }
        else
        {
            delete agent;
            duplicates += 1;
        }
    }
    if (duplicates > 0)
    {
        std::cout << "Note: removed " << duplicates << " duplicates from scenario." << std::endl;
    }
    agents = std::vector<Ped::Tagent *>(agentsWithUniquePosition.begin(), agentsWithUniquePosition.end());
}

vector<Ped::Tagent *> ParseScenario::getAgents() const
{
    return agents;
}

void **ParseScenario::getPositionArrays() const
{
    return positionArrays;
}

std::vector<Ped::Twaypoint *> ParseScenario::getWaypoints()
{
    std::vector<Ped::Twaypoint *> v; //
    for (auto p : waypoints)
    {
        v.push_back((p.second));
    }
    return std::move(v);
}
