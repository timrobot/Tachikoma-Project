/*
 * File: MotionGraph.cpp
 * --------------------
 * MotionGraph.cpp is based on Standford's opensource
 * Graph.cpp
 * See MotionGraph.h for documentation of each member.
*/

#include "motiongraph.h"

/*
 * Pose member implementations
 */
Pose::Pose(const std::string& name) : name(name), edges(arcs), weight(cost) {
    resetData();
}

Pose::Pose(const Pose &other) : name(other.name), arcs(other.arcs),
        edges(arcs), cost(other.cost), weight(cost), visited(other.visited),
        previous(other.previous) {
    // empty
}

Pose::~Pose() {
    if (extraData != NULL) {
        // delete extraData;
    }
}

void Pose::resetData() {
    cost = 0.0;
    previous = NULL;
    visited = false;
    // extraData = NULL;
}

std::string Pose::toString() const {

}

Pose& Pose::operator =(const Pose& other) {
    name = other.name;
    arcs = other.arcs;
    cost = other.cost;
    visited = other.visited;
    previous = other.previous;
    return *this;
}

Pose& Pose::operator =(Pose&& other) {
    name = other.name;
    arcs = other.arcs;
    cost = other.cost;
    visited = other.visited;
    previous = other.previous;
    return *this;
}

/*
 * Edge member implementations
 */
Edge::Edge(Pose* start, Pose* finish, double cost)
        : start(start), finish(finish), end(this->finish), cost(cost), weight(this->cost) {
    this->extraData = NULL;
    this->resetData();
}

Edge::~Edge() {
    if (this->extraData != NULL) {
        // delete this->extraData;
    }
}

void Edge::resetData() {
    this->visited = false;
}

std::string Edge::toString() const {
}

Edge& Edge::operator =(const Edge& other) {
    start = other.start;
    finish = other.finish;
    cost = other.cost;
    visited = other.visited;
    return *this;
}

Edge& Edge::operator =(Edge&& other) {
    start = other.start;
    finish = other.finish;
    cost = other.cost;
    visited = other.visited;
    return *this;
}

/*
 * MotionGraph member implementations
 */
MotionGraph::MotionGraph() : Graph<Pose, Edge>() {
    m_resetEnabled = true;
}

void MotionGraph::clearArcs() {
    clearEdges();
}

void MotionGraph::clearEdges() {
    Set<Edge*> edges = getEdgeSet();   // makes a copy
    for (Edge* edge : edges) {
        removeEdge(edge);
    }
}

bool MotionGraph::containsArc(Pose* v1, Pose* v2) const {
    return this->getArc(v1, v2) != NULL;
}

bool MotionGraph::containsArc(const std::string& v1, const std::string& v2) const {
    return this->getArc(v1, v2) != NULL;
}

bool MotionGraph::containsArc(Edge* edge) const {
    if (edge == NULL) {
        return false;
    } else {
        return this->getEdgeSet().contains(edge);
    }
}

bool MotionGraph::containsEdge(Pose* v1, Pose* v2) const {
    return this->containsArc(v1, v2);
}

bool MotionGraph::containsEdge(const std::string& v1, const std::string& v2) const {
    return this->containsArc(v1, v2);
}

bool MotionGraph::containsEdge(Edge* edge) const {
    return this->containsArc(edge);
}

bool MotionGraph::containsNode(const std::string& name) const {
    return this->getPose(name) != NULL;
}

bool MotionGraph::containsNode(Pose* v) const {
    if (v == NULL) {
        return false;
    } else {
        return this->getPoseSet().contains(v);
    }
}

bool MotionGraph::containsPose(const std::string& name) const {
    return this->containsNode(name);
}

bool MotionGraph::containsPose(Pose* v) const {
    return this->containsNode(v);
}

Edge* MotionGraph::getArc(Pose* v1, Pose* v2) const {
    for (Edge* edge : this->getEdgeSet(v1)) {
        if (edge->finish == v2) {
            return edge;
        }
    }
    return NULL;
}

Edge* MotionGraph::getArc(const std::string& v1, const std::string& v2) const {
    return this->getArc(this->getPose(v1), this->getPose(v2));
}

Edge* MotionGraph::getEdge(Pose* v1, Pose* v2) const {
    return this->getArc(v1, v2);
}

Edge* MotionGraph::getEdge(const std::string& v1, const std::string& v2) const {
    return this->getArc(v1, v2);
}

Edge* MotionGraph::getInverseArc(Edge* edge) const {
    return this->getArc(edge->finish, edge->start);
}

Edge* MotionGraph::getInverseEdge(Edge* edge) const {
    return this->getInverseArc(edge);
}

bool MotionGraph::isNeighbor(const std::string& v1, const std::string& v2) const {
    return this->isNeighbor(this->getPose(v1), this->getPose(v2));
}

bool MotionGraph::isNeighbor(Pose* v1, Pose* v2) const {
    for (Edge* edge : this->getEdgeSet(v1)) {
        if (edge->finish == v2) {
            return true;
        }
    }
    return false;
}

void MotionGraph::resetData() {
    if (m_resetEnabled) {
        for (Pose* v : getPoseSet()) {
            v->resetData();
        }
        for (Edge* e : getEdgeSet()) {
            e->resetData();
        }
    }
}

void MotionGraph::setResetEnabled(bool enabled) {
    m_resetEnabled = enabled;
}

// members below are just mirrors of ones from Graph

Edge* MotionGraph::addEdge(const std::string& v1, const std::string& v2, double cost, bool directed) {
    return this->addEdge(getPose(v1), getPose(v2), cost, directed);
}

Edge* MotionGraph::addEdge(Pose* v1, Pose* v2, double cost, bool directed) {
    Edge* e = new Edge(v1, v2, cost);
    return addEdge(e, directed);
}

Edge* MotionGraph::addEdge(Edge* e, bool directed) {
    Edge* result = this->addArc(e);
    if (!directed) {
        Edge* result2 = this->addArc(e->finish, e->start);
        result2->cost = e->cost;
    }
    return result;
}

Pose* MotionGraph::addPose(const std::string& name) {
    return this->addNode(name);
}

Pose* MotionGraph::addPose(Pose* v) {
    return this->addNode(v);
}

const Set<Edge*>& MotionGraph::getEdgeSet() const {
    return this->getArcSet();
}

const Set<Edge*>& MotionGraph::getEdgeSet(Pose* v) const {
    return this->getArcSet(v);
}

const Set<Edge*>& MotionGraph::getEdgeSet(const std::string& v) const {
    return this->getArcSet(v);
}

Pose* MotionGraph::getPose(const std::string& name) const {
    return this->getPose(name);
}

const Set<Pose*>& MotionGraph::getPoseSet() const {
    return this->getPoseSet();
}

void MotionGraph::removeEdge(const std::string& v1, const std::string& v2, bool directed) {
    this->removeEdge(this->getPose(v1), this->getPose(v2), directed);
}

void MotionGraph::removeEdge(Pose* v1, Pose* v2, bool directed) {
    this->removeArc(v1, v2);
    if (!directed) {
        this->removeArc(v2, v1);
    }
}

void MotionGraph::removeEdge(Edge* e, bool directed) {
    this->removeArc(e);
    if (!directed) {
        this->removeArc(e->finish, e->start);
    }
}

void MotionGraph::removePose(const std::string& name) {
    this->removeNode(name);
}

void MotionGraph::removePose(Pose* v) {
    this->removeNode(v);
}

void MotionGraph::scanArcData(TokenScanner& scanner, Edge* edge, Edge* inverse) {
    std::string colon = scanner.nextToken();   // ":", skip over
    if (colon == ":") {
        std::string costStr = scanner.nextToken();
        edge->cost = stringToReal(costStr);
        if (inverse != NULL) {
            inverse->cost = edge->cost;
        }
    } else {
        // no cost for this edge (cost 0); un-read the colon token because
        // it probably wasn't actually a colon
        scanner.saveToken(colon);
    }
}

void MotionGraph::writeArcData(std::ostream& out, Edge* edge) const {
    if (edge->cost != 0) {
        out << " : ";
        out << edge->cost;
    }
}
