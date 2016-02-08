/*
 * File: motiongraph.h
 * ------------------
 * MotionGraph.cpp is based on Standford's opensource
 * graph.c by Marty Stepp
 */

#ifndef _motiongraph_h
#define _motiongraph_h

#include <string>
#include "graph.h"
#include "grid.h"
#include "observable.h"
#include "set.h"

/*
 * Forward declarations of Vertex/Edge structures so that they can refer
 * to each other mutually.
 */
struct Vertex;
struct Edge;

/*
 * Canonical Vertex (Node) structure implementation needed by Graph class template.
 * Each Vertex structure represents a single vertex in the graph.
 */
struct Vertex : public Observable {
public:
    std::string name;    // required by Stanford Graph; vertex's name
    Set<Edge*> arcs;     // edges outbound from this vertex; to neighbors
    Set<Edge*>& edges;   // alias of arcs; preferred name

    /*
     * The following three fields are 'supplementary data' inside each vertex.
     * You can use them in your path-searching algorithms to store various
     * information related to the vertex
     */
    double cost;        // cost to reach this vertex (initially 0; you can set this)
    double& weight;     // alias of cost; they are the same field
    bool visited;       // whether this vertex has been visited before (initally false; you can set this)
    Vertex* previous;   // vertex that comes before this one (initially NULL; you can set this)

    /*
     * The following pointer can point to any extra data needed by the vertex.
     * This field is generally not necessary and can be ignored.
     */
    void* extraData;

    /*
     * Constructs a vertex with the given name.
     */
    Vertex(const std::string& name = "");

    /*
     * Copy constructor (rule of three).
     */
    Vertex(const Vertex& other);

    /*
     * Frees up any memory dynamically allocated by this vertex.
     */
    ~Vertex();

    /*
     * Wipes the supplementary data of this vertex back to its initial state.
     * Specifically, sets cost to 0, visited to false, and previous to NULL.
     */
    void resetData();

    /*
     * Returns a string representation of this vertex for debugging, such as
     * "Vertex{name=r13c42, cost=11, visited=true, previous=r12c41, neighbors={r12c41, r12c43}}".
     */
    std::string toString() const;

    /*
     * Copy assignment operator (rule of three).
     */
    Vertex& operator =(const Vertex& other);

    /*
     * Move assignment operator (rule of three).
     */
    Vertex& operator =(Vertex&& other);

private:
   
};

/*
 * You can refer to a Vertex as a Node if you prefer.
 */
#define Node Vertex

/*
 * Canonical Edge (Arc) structure implementation needed by Graph class template.
 * Each Edge structure represents a single edge in the graph.
 */
struct Edge {
public:
    Vertex* start;    // edge's starting vertex (required by Graph)
    Vertex* finish;   // edge's ending vertex (required by Graph)
    Vertex*& end;     // alias of finish; they are the same field
    double cost;      // edge weight (required by Graph)
    double& weight;   // alias of cost; they are the same field
    bool visited;     // whether this edge has been visited before (initally false; you can set this)

    /*
     * The following pointer can point to any extra data needed by the vertex.
     * This field is generally not necessary and can be ignored.
     */
    void* extraData;

    /*
     * Constructs a new edge between the given start/end vertices with
     * the given cost.
     */
    Edge(Vertex* start = NULL, Vertex* finish = NULL, double cost = 0.0);

    /*
     * Frees up any memory dynamically allocated by this edge.
     */
    ~Edge();

    /*
     * Wipes the supplementary data of this vertex back to its initial state.
     * Specifically, sets visited to false.
     */
    void resetData();

    /*
     * Returns a string representation of this edge for debugging, such as
     * "Arc{start=r12c42, finish=r12c41, cost=0.75}".
     */
    std::string toString() const;
    
    /*
     * Copy assignment operator (rule of three).
     */
    Edge& operator =(const Edge& other);

    /*
     * Move assignment operator (rule of three).
     */
    Edge& operator =(Edge&& other);
};

#define Arc Edge

class MotionGraph : public Graph<Vertex, Edge> {
public:
    /*
     * Newly added behavior in MotionGraph.
     */
    MotionGraph();
    void clearArcs();
    void clearEdges();
    bool containsArc(Vertex* v1, Vertex* v2) const;
    bool containsArc(const std::string& v1, const std::string& v2) const;
    bool containsArc(Edge* edge) const;
    bool containsEdge(Vertex* v1, Vertex* v2) const;
    bool containsEdge(const std::string& v1, const std::string& v2) const;
    bool containsEdge(Edge* edge) const;
    bool containsNode(const std::string& name) const;
    bool containsNode(Vertex* v) const;
    bool containsVertex(const std::string& name) const;
    bool containsVertex(Vertex* v) const;
    Edge* getArc(Vertex* v1, Vertex* v2) const;
    Edge* getArc(const std::string& v1, const std::string& v2) const;
    Edge* getEdge(Vertex* v1, Vertex* v2) const;
    Edge* getEdge(const std::string& v1, const std::string& v2) const;
    Edge* getInverseArc(Edge* edge) const;
    Edge* getInverseEdge(Edge* edge) const;
    bool isNeighbor(const std::string& v1, const std::string& v2) const;
    bool isNeighbor(Vertex* v1, Vertex* v2) const;
    void resetData();
    void setResetEnabled(bool enabled);
    virtual void scanArcData(TokenScanner& scanner, Edge* edge, Edge* inverse);
    virtual void writeArcData(std::ostream& out, Edge* edge) const;

    Edge* addEdge(const std::string& v1, const std::string& v2, double cost = 0.0, bool directed = true);
    Edge* addEdge(Vertex* v1, Vertex* v2, double cost = 0.0, bool directed = true);
    Edge* addEdge(Edge* e, bool directed = true);
    Vertex* addVertex(const std::string& name);
    Vertex* addVertex(Vertex* v);
    const Set<Edge*>& getEdgeSet() const;
    const Set<Edge*>& getEdgeSet(Vertex* v) const;
    const Set<Edge*>& getEdgeSet(const std::string& v) const;
    Vertex* getVertex(const std::string& name) const;
    const Set<Vertex*>& getVertexSet() const;
    void removeEdge(const std::string& v1, const std::string& v2, bool directed = true);
    void removeEdge(Vertex* v1, Vertex* v2, bool directed = true);
    void removeEdge(Edge* e, bool directed = true);
    void removeVertex(const std::string& name);
    void removeVertex(Vertex* v);

private:
    bool m_resetEnabled;
};

#endif