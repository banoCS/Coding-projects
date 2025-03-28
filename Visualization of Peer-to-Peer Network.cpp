#include <iostream>
#include <map>
#include <vector>
#include <graphics.h>
#include <cmath>
#include <stdexcept> // For exception handling
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

// BaseNode class to serve as a base class for different types of nodes
class BaseNode {
public:
    virtual ~BaseNode() {}

    // Pure virtual function to get IP address
    virtual string getIpAddress() const = 0;

    // Virtual function to display node type
    virtual void displayNodeType() const {
        cout << "BaseNode" << endl;
    }
};

// Node class derived from BaseNode to represent each peer in the network
class Node : public BaseNode {
private:
    string ipAddress; // Encapsulation: Private data member

public:
    // Constructor: Initializes a Node with an IP address
    Node(const string& ip) : ipAddress(ip) {}

   // Override method to get the IP address
    string getIpAddress() const override {
        return ipAddress;
    }

    // Override method to display node type
    void displayNodeType() const override {
        cout << "Node" << endl;
    }
};

// Graph class to manage the nodes and network operations
class Graph {
//private:
    int V; // Number of vertices
    vector<vector<int> > adjMatrix; // Adjacency matrix

public:
    Graph(int V) {
        this->V = V;
        adjMatrix.resize(V, vector<int>(V, 0));
    }

    void addEdge(int u, int v, int w) {
        adjMatrix[u][v] = w;
        adjMatrix[v][u] = w; // For undirected graph
    }

    void dijkstra(int src) {
        vector<int> dist(V, INT_MAX); // Distance array
        vector<bool> sptSet(V, false); // Shortest path tree set

        dist[src] = 0; // Distance of source vertex is 0

        for (int count = 0; count < V - 1; count++) {
            int u = minDistance(dist, sptSet);
            sptSet[u] = true;

            for (int v = 0; v < V; v++) {
                if (!sptSet[v] && adjMatrix[u][v] && dist[u] != INT_MAX && dist[u] + adjMatrix[u][v] < dist[v]) {
                    dist[v] = dist[u] + adjMatrix[u][v];
                }
            }
        }

        printSolution(dist);
    }

    int minDistance(const vector<int>& dist, const vector<bool>& sptSet) {
        int min = INT_MAX, min_index = -1;
        for (int v = 0; v < V; v++) {
            if (!sptSet[v] && dist[v] <= min) {
                min = dist[v];
                min_index = v;
            }
        }
        return min_index;
    }

    void printSolution(const vector<int>& dist) {
        cout << "Vertex \t Distance from Source/ cost " << endl;
        for (int i = 0; i < V; i++) {
            cout << i << " \t\t " << dist[i] << endl;
        }
    }

    void inputEdges() {
        int E;
        cout << "Enter the number of edges: ";
        cin >> E;

        for (int i = 0; i < E; i++) {
            int u, v, w;
            cout << "Enter edge " << i + 1 << "(u, v, w): ";
            cin >> u >> v >> w;
            addEdge(u, v, w);
        }
    }
};

// Network class to manage the nodes and network operations
class Network {
private:
    map<char, BaseNode*> nodes; // Association: Map to associate node ID with BaseNode objects
    Graph* graph; // Graph for managing edges and Dijkstra's algorithm

public:
    // Constructor: Initializes the network with 4 nodes
    Network(int V) : graph(new Graph(V)) {
        nodes['A'] = new Node("192.168.1.1");
        nodes['B'] = new Node("192.168.1.2");
        nodes['C'] = new Node("192.168.1.3");
        nodes['D'] = new Node("192.168.1.4");
    
    }

    // Destructor: Cleans up dynamically allocated BaseNode objects
    ~Network() {
        for (map<char, BaseNode*>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
            delete it->second;
        }
        delete graph;
    }

    // Method to add a new node to the network
    void addNode(char id, const string& ipAddress) {
        try {
            if (nodes.size() < 10 && nodes.find(id) == nodes.end()) {
                nodes[id] = new Node(ipAddress);
                cout << "Node " << id << " added with IP Address: " << ipAddress << endl;
            } else {
                throw runtime_error("Cannot add more nodes or node already exists.");
            }
        } catch (const exception& e) {
            cout << e.what() << endl;
        }
    }

    // Method to remove a node from the network
    void removeNode(char id) {
        try {
            map<char, BaseNode*>::iterator it = nodes.find(id);
            if (it != nodes.end()) {
                delete it->second;
                nodes.erase(it);
                cout << "Node " << id << " removed." << endl;
            } else {
                throw runtime_error(string("Node ") + id + " does not exist.");
            }
        } catch (const exception& e) {
            cout << e.what() << endl;
        }
    }

    // Method to display all nodes in the network
    void displayNodes() const {
        for (map<char, BaseNode*>::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {
            cout << "Node " << it->first << " IP Address: " << it->second->getIpAddress() << " Type: ";
            it->second->displayNodeType();
        }
    }

    // Method to determine and display the most cost-effective network topology
    void displayTopology() const {
        if (nodes.size() <= 5) {
            cout << "A bus topology is cost-effective for small networks." << endl;
        } else {
            cout << "A mesh topology is cost-effective for larger networks." << endl;
        }
    }

    // Method to send a "Hello" message between nodes
    void sendMessage(char senderId, char receiverId) const {
        try {
            if (nodes.find(senderId) != nodes.end() && nodes.find(receiverId) != nodes.end()) {
                cout << "Node " << senderId << " sent 'Hello' to Node " << receiverId << endl;
                animateMessage(senderId, receiverId);
            } else {
                throw runtime_error("One or both nodes do not exist.");
            }
        } catch (const exception& e) {
            cout << e.what() << endl;
        }
    }

    // Method to draw the network topology
    void drawTopology() const {
    
        initwindow(600,600);  //height and width
    
        int numNodes = nodes.size();
        if (numNodes > 0 && numNodes <= 5) {
            drawBusTopology(numNodes);
        } else if (numNodes > 5) {
            drawMeshTopology(numNodes, 300, 300, 20, 60);
        } else {
            cout << "Number of nodes should be 1 or more for this demonstration." << endl;
        }

        closegraph(30000);
    }

    // Method to manage edges for the graph
    void inputEdges() {
        graph->inputEdges();
    }

    // Method to execute Dijkstra's algorithm
    void runDijkstra(int src) {
        graph->dijkstra(src);
    }

private:
    // Function to draw a circle representing a node
    void drawNode(int x, int y, int radius, const char* label) const {
        setcolor(YELLOW);
        setfillstyle(SOLID_FILL, WHITE);
        circle(x, y, radius);
        floodfill(x, y, YELLOW);
        setcolor(LIGHTCYAN);
        int text_width = textwidth(const_cast<char*>(label)); // Casting const char* to char*
        int text_height = textheight(const_cast<char*>(label)); // Casting const char* to char*
        outtextxy(x - text_width / 2, y - text_height / 2, const_cast<char*>(label)); // Casting const char* to char*
    }

    // Method to draw a bus topology with the specified number of nodes
    void drawBusTopology(int numNodes) const {
        int width = getmaxx();
        int height = getmaxy();

        // Calculate the positions
        int busY = height / 4;
        int startX = 70;
        int endX = width - 50;
        int nodeSpacing = (endX - startX) / (numNodes - 1);

        // Draw the bus line
        line(startX, busY, endX, busY);

        // Draw the nodes and connect them to the bus line
        for (int i = 0; i < numNodes; ++i) {
            int nodeX = startX + i * nodeSpacing;
            char label[2] = {char('A' + i), '\0'};
            drawNode(nodeX, busY - 50, 20, label); // Draw the node above the bus line
            line(nodeX, busY - 10, nodeX, busY); // Connect node to the bus line
        }
    }

    // Function to draw a mesh topology with the specified number of nodes
    void drawMeshTopology(int numNodes, int centralX, int centralY, int nodeRadius, int spaceBetween) const {
        int radius = nodeRadius + spaceBetween;
        double angleIncrement = 2 * M_PI / numNodes;
        double angle = -M_PI / 2;

        int nodeX[numNodes], nodeY[numNodes];

        // Draw the nodes in a circular pattern
        for (int i = 0; i < numNodes; ++i) {
            nodeX[i] = centralX + radius * cos(angle);
            nodeY[i] = centralY + radius * sin(angle);
            char label[2] = {char('A' + i), '\0'};
            drawNode(nodeX[i], nodeY[i], nodeRadius, label);
            angle += angleIncrement;
        }

        // Connect each node to every other node
        for (int i = 0; i < numNodes; ++i) {
            for (int j = i + 1; j < numNodes; ++j) {
                line(nodeX[i], nodeY[i], nodeX[j], nodeY[j]);
            }
        }
    }

    // Function to draw a message between two points
   
    void drawMessage(int x, int y, const char* message) const {
        setcolor(BLACK);
        setfillstyle(SOLID_FILL, CYAN);
        rectangle(x - 30, y - 10, x + 30, y + 10);
        floodfill(x, y, BLACK);
        setcolor(GREEN);
        outtextxy(x - 25, y - 5, const_cast<char*>(message)); 
    }

    // Function to animate a message from one node to another
    void animateMessage(char senderId, char receiverId) const {
       
        initwindow(900,1000);

        map<char, pair<int, int> > nodePositions;

        int nodeCount = 0;
        int centerX = getmaxx() / 2;
        int centerY = getmaxy() / 2;
        int radius = min(centerX, centerY) - 100;
        double angleIncrement = 2 * M_PI / nodes.size();
        double angle = -M_PI / 2;

        // Calculate positions for all nodes
        for (auto it = nodes.begin(); it != nodes.end(); ++it) {
            int x = centerX + radius * cos(angle);
            int y = centerY + radius * sin(angle);
            nodePositions[it->first] = {x, y};
            angle += angleIncrement;
            nodeCount++;
        }

        int x1 = nodePositions[senderId].first;
        int y1 = nodePositions[senderId].second;
        int x2 = nodePositions[receiverId].first;
        int y2 = nodePositions[receiverId].second;

// Calculate intermediate point to create a smoother path
    int midX = (x1 + x2) / 2;
    int midY = (y1 + y2) / 2;
        for (int i = 0; i <= 100; i += 2) {
            cleardevice();

            // Draw nodes
            for (map<char, pair<int, int> >::iterator it = nodePositions.begin(); it != nodePositions.end(); ++it) {
                char label[2] = {it->first, '\0'};
                drawNode(it->second.first, it->second.second, 25, label);
            }

            // Animate message
            int x = x1 + (x2 - x1) * i / 100;
            int y = y1 + (y2 - y1) * i / 100;
            drawMessage(x, y, "HELLO");

            delay(40);
        }

        closegraph();
    }
};

// Function to display the banner with available commands
void banner() {
    cout << "\t\t\t\t@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
    cout << "\t\t\t\t        Welcome to the Network Management System" << endl;
    cout << "\t\t\t\t@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl << endl << endl << endl;

    cout << "Enter command to select operation" << endl;
    cout << "A: Add Node" << endl;
    cout << "R: Remove Node" << endl;
    cout << "D: Display Nodes" << endl; 
    cout << "T: Display Topology" << endl;
    cout << "S: Send Message" << endl; 
    cout << "E: Edit Edges" << endl; 
    cout << "J: Run Dijkstra" << endl;
    cout << "Q: Quit" << endl;
}

int main() {
    banner();
    Network network(10); // Assuming 5 is the maximum number of nodes

    char command, nodeId, targetId;
    string ipAddress;
    int srcNode;

    while (true) {
        cout << "Enter command: ";
        cin >> command;

        switch (command) {
            case 'A':
                cout << "Enter Node ID and IP Address to add: ";
                cin >> nodeId >> ipAddress;
                network.addNode(nodeId, ipAddress);
                break;

            case 'R':
                cout << "Enter Node ID to remove: ";
                cin >> nodeId;
                network.removeNode(nodeId);
                break;

            case 'D':
                network.displayNodes();
                break;

            case 'T':
                network.displayTopology();
                network.drawTopology();
                break;

            case 'S':
                cout << "Enter Sender Node ID and Receiver Node ID: ";
                cin >> nodeId >> targetId;
                network.sendMessage(nodeId, targetId);
                break;

            case 'E':
                network.inputEdges();
                break;

            case 'J':
                cout << "Enter source node index for Dijkstra: ";
                cin >> srcNode;
                network.runDijkstra(srcNode);
                break;

            case 'Q':
                cout << "Exiting the Network Management System." << endl;
                return 0;

            default:
                cout << "Invalid command. Please try again." << endl;
        }
    }

    return 0;
}
