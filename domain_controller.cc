#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <string>
#include <math.h>
#include <vector>
#include <algorithm>
#include <route.h>
#include "switch_message_m.h"
#include "condition.h"
#include "node.h"

using namespace omnetpp;
using namespace std;

// --- QUpdate class to hold delta updates for the Q-table ---
class QUpdate : public cObject {
  public:
    double delta[20][20][20];
    QUpdate() { memset(delta, 0, sizeof(delta)); }
    virtual QUpdate* dup() const override { return new QUpdate(*this); }
};



class domain_controller : public cSimpleModule
{
  public:
    // Network condition matrices
    double loss[20][20];
    double transmissionDelay[20][20];
    double queuingDelay[20][20];
    double availableBandwidth[20][20];
    double totalBandwidth[20][20];
    int get;          // Number of slave controllers that have sent network condition
    int src, des;     // Current source and destination for the routing episode
    int nex[20][20];  // Forward table (e.g., nex[state][dest])
    int num;          // Number of switches under control
    bool isIn[20];    // Whether a switch is under this controller’s control
    bool G[20][20];
    bool turnIn;

    // Reinforcement Learning parameters
    int visit[20][20];
    double Q[20][20][20];  // Q[dest][state][nextHop]
    double epsilon = 0.1, tau0 = 100, tauT = 0.5, T = 100;
    double beta1 = 1, beta2 = 1, beta3 = 1, theta1 = 1, phi1 = 1;
    double theta2 = 0.5, phi2 = 0.5;
    double alpha = 1, gamma = 1;
    double pi = acos(-1.0);

    // Federated update members:
    double Q_prev[20][20][20];  // Stored copy of the Q table
    double federatedRate;       // Federated learning rate (value in [0,1])
    
  protected:
    virtual void getIsIn();
    virtual double getTau();
    virtual void finish() override;
    virtual double getReward(int state, int nextState);
    virtual void forwardMessageToSlave(cMessage* msg);
    virtual void forwardMessageToSwitch(); // if needed
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void copyCondition(condition* cond);
    virtual void retrieveCondition(cMessage* msg);
    virtual void forwardMessage(int to, int nextHop, string name);
    virtual int makeSoftmaxPolicy(int state, double (& q)[20]);
    virtual void sarsa(double (& q)[20][20]);
};

Define_Module(domain_controller);

double domain_controller::getTau() {
    double tauN = -((tau0 - tauT) * visit[src][des]) / T + tau0;
    printf("[domain%d:getTau] visit[%d][%d] = %d, tau = %f.\n", getIndex(), src, des, visit[src][des], tauN);
    return tauN;
}

namespace temp{
    bool cmp(Node* a, Node* b) {
        return a->val > b->val;
    }
}

int domain_controller::makeSoftmaxPolicy(int state, double (& q)[20]) {
    printf("[domain%d:makeSoftmaxPolicy] Executing softmax policy for state %d.\n", getIndex(), state);
    vector<Node*> vec; // record the id of the surrounding nodes
    for (int i = 0; i < 20; ++i) {
        if (G[state][i] && isIn[i])
            vec.push_back(new Node(i, q[i]));
    }
    int len = vec.size();
    double tauN = getTau();
    double sum = 0;

    for (int i = 0; i < len; ++i) {
        vec[i]->val = exp(vec[i]->val / tauN);
        sum += vec[i]->val;
    }
    for (int i = 0; i < len; ++i)
    vec[i]->val /= sum;

    printf("[domain%d:makeSoftmaxPolicy] Computed probabilities for %d actions.\n", getIndex(), len);
    sort(vec.begin(), vec.end(), temp::cmp);
    double chance = uniform(0, 1);
    for (int i = 0; i < len; ++i) {
        if (chance <= vec[i]->val) {
            std::cout << "[domain:softmax] propability of " << vec[i]->idx  << " value: " << vec[i]->val << std::endl;
            return vec[i]->idx;
        }
        else
            chance -= vec[i]->val;
    }
    printf("[domain%d:makeSoftmaxPolicy] Default selected next hop %d.\n", getIndex(), vec[len - 1]->idx);
    return vec[len - 1]->idx;
}

double domain_controller::getReward(int state, int nextState) {
    double totalQueuingDelay = 0;
    double totalTransmissionDelay = 0;
    double totalAvailableBandwidth = 0;
    int n = 0; // number of the surrounding nodes
    for (int i = 0; i < 20; ++i) {
        if (isIn[i] && G[state][i]) {
            ++n;
            totalQueuingDelay += queuingDelay[state][i];
            totalTransmissionDelay += transmissionDelay[state][i];
            totalAvailableBandwidth += availableBandwidth[state][i];
        }
    }
    double delay = 2 * atan(transmissionDelay[state][nextState] - totalTransmissionDelay / n) / pi;
    double queue = 2 * atan(queuingDelay[state][nextState] - totalQueuingDelay / n) / pi;
    double los = 1 - 2 * loss[state][nextState];
    double B1 = 2 * availableBandwidth[state][nextState] / totalBandwidth[state][nextState];
    double B2 = 2 * atan(0.01 * (availableBandwidth[state][nextState] - totalAvailableBandwidth / n)) / pi;
    double reward = -3 + beta1 * (theta1 * delay + theta2 * queue) + beta2 * los + beta3 * (phi1 * B1 + phi2 * B2);
    printf("[domain%d:getReward] For state %d, nextState %d, reward = %f.\n", getIndex(), state, nextState, reward);
    return reward;
}

void domain_controller::sarsa(double (& q)[20][20]) {
    ++visit[src][des];
    if (visit[src][des] == 1) {
        for (int i = 0; i < 20; ++i)
            for (int j = 0; j < 20; ++j)
                q[i][j] = 0;
    }
    if (visit[src][des] == 101)
        visit[src][des] = 1;
    int state = src, action, nextState, nextAction;
    action = makeSoftmaxPolicy(state, q[state]);
    double reward;
    printf("[domain%d:sarsa] Running SARSA for route from %d to %d.\n", getIndex(), src, des);
    while (true) {
        nex[state][des] = action;
        nextState = action;
        nextAction = makeSoftmaxPolicy(nextState, q[nextState]);
        reward = getReward(state, action);
        printf("[domain%d:sarsa] state %d, action %d, nextState %d, nextAction %d, reward %f.\n",
               getIndex(), state, action, nextState, nextAction, reward);
        q[state][action] += alpha * (reward + gamma * q[nextState][nextAction] - q[state][action]);
        printf("[domain%d:sarsa] Updated Q[%d][%d] = %f.\n", getIndex(), state, action, q[state][action]);
        if (nextState == des)
            break;
        action = nextAction;
        state = nextState;
    }
    printf("[domain%d:sarsa] Completed SARSA. visit[%d][%d] = %d.\n", getIndex(), src, des, visit[src][des]);

    // --- Federated Update Step ---
    // Compute the delta between current Q and the stored Q_prev,
    // scale it by federatedRate, and send it to the super controller.
    QUpdate *updateObj = new QUpdate();
    
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            for (int k = 0; k < 20; ++k) {
                double diff = Q[i][j][k];
                updateObj->delta[i][j][k] = federatedRate * diff;
            }
        }
    }
    cMessage *updateMsg = new cMessage("Federated update");
    updateMsg->addObject(updateObj);
    send(updateMsg, "super$o");  // Using the "super" gate as defined in your NED
    // Update the local stored Q_prev with current Q.
    memcpy(Q_prev, Q, sizeof(Q));
}

void domain_controller::getIsIn() {
    memset(isIn, 0, sizeof(isIn));
    num = 0;
    bool flag;
    for (int i = 0; i < 20; ++i) {
        flag = false;
        for (int j = 0; j < 20; ++j)
            if (G[i][j]) {
                flag = true;
                break;
            }
        isIn[i] = flag;
        num += flag;
    }
    printf("[domain%d:getIsIn] Number of switches under control: %d.\n", getIndex(), num);
}

void domain_controller::initialize() {
    printf("[domain%d:initialize] Initializing domain controller.\n", getIndex());
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            loss[i][j] = -1;
            transmissionDelay[i][j] = -1;
            queuingDelay[i][j] = -1;
            availableBandwidth[i][j] = -1;
            totalBandwidth[i][j] = -1;
            G[i][j] = false;
            nex[i][j] = -1;
            visit[i][j] = 0;
        }
    }
    get = 0;
    memset(Q, 0, sizeof(Q));
    memset(Q_prev, 0, sizeof(Q_prev));
    federatedRate = 0.99; // Set federated learning rate here.
    getIsIn();
    printf("[domain%d:initialize] Initialization complete.\n", getIndex());
}

void domain_controller::forwardMessage(int to, int nextHop, string name) {
    printf("[domain%d:forwardMessage] Sending message '%s' to switch %d with nextHop %d (destination %d).\n", getIndex(), name.c_str(), to, nextHop, des);
    for (cModule::GateIterator i(this); !i.end(); i++) {
         cGate *gate = i();
         std::string gateStr = gate->getName();
         if (gateStr == "gate$o" && gate->getPathEndGate()->getOwnerModule()->getIndex() == to) {
             int senderId = gate->getIndex();
             cMessage* msg = new cMessage(name.c_str());
             msg->addPar("hop").setLongValue(nextHop);
             msg->addPar("des").setLongValue(des);
             send(msg->dup(), "gate$o", senderId);
         }
    }
}

void domain_controller::forwardMessageToSwitch() {
    int now = src;
    printf("[domain%d:forwardMessageToSwitch] Forwarding route plan from src %d to des %d.\n", getIndex(), src, des);
    while (now != des) {
        if (isIn[now]) {
            printf("[domain%d:forwardMessageToSwitch] Updating switch %d with nextHop %d.\n", getIndex(), now, nex[now][des]);
            forwardMessage(now, nex[now][des], "update");
        }
        now = nex[now][des];
    }
    if (isIn[src]) {
        printf("[domain%d:forwardMessageToSwitch] Informing source switch %d to send message according to route plan.\n", getIndex(), src);
        forwardMessage(src, 0, "send");
    }
}

void domain_controller::handleMessage(cMessage *msg) {
    printf("[domain%d:handleMessage] Received message '%s' from '%s'.\n", getIndex(), msg->getName(), msg->getSenderModule()->getName());
    string from = msg->getSenderModule()->getName();
    string name = msg->getName();
    if (from == "switches") {
        turnIn = true;
        printf("[domain%d:handleMessage] Received message from switches. Setting turnIn = true and forwarding to slave.\n", getIndex());
        forwardMessageToSlave(msg);
    } else if (from == "slave") {
        get++;
        printf("[domain%d:handleMessage] Received message '%s' from slave. get = %d.\n", getIndex(), msg->getName(), get);
        retrieveCondition(msg);
        if (get == 2) { // all network information received
            get = 0;
            if (isIn[src] && isIn[des]) {
                printf("[domain%d:handleMessage] All network info received. Running SARSA for route from %d to %d.\n", getIndex(), src, des);
                sarsa(Q[des]);
                int l = src;
                printf("[domain%d:handleMessage] Calculated route: ", getIndex());
                while (l != des) {
                    printf("%d -> ", l);
                    l = nex[l][des];
                }
                printf("%d\n", des);
                forwardMessageToSwitch();
            } else {
                if (turnIn) {
                    turnIn = false;
                    switch_message* msg_ = new switch_message("turn in");
                    msg_->setSource(src);
                    msg_->setDestination(des);
                    printf("[domain%d:handleMessage] Not all switches under control. Forwarding message to super controller.\n", getIndex());
                    send(msg_->dup(), "super$o");
                } else {
                    cMessage* mmsg = new cMessage("retrieve");
                    condition* cond = new condition();
                    copyCondition(cond);
                    mmsg->addObject(cond);
                    printf("[domain%d:handleMessage] Forwarding retrieve request with condition to super controller.\n", getIndex());
                    send(mmsg, "super$o");
                }
            }
        }
    } else if (from == "super") {
        if (name == "request") {
            printf("[domain%d:handleMessage] Received 'request' from super. Forwarding to slave.\n", getIndex());
            forwardMessageToSlave(msg);
        } else if (name == "update") {
            Route* route = (Route*) msg->getObject("");
            des = msg->par("des").longValue();
            src = msg->par("src").longValue();
            for (int i = 0; i < 20; ++i)
                nex[i][des] = route->nex[i][des];
            printf("[domain%d:handleMessage] Received 'update' from super. Updated route plan from src %d to des %d.\n", getIndex(), src, des);
            forwardMessageToSwitch();
        }
    }
}

void domain_controller::forwardMessageToSlave(cMessage* msg) {
    switch_message* tempmsg = check_and_cast<switch_message *>(msg);
    src = tempmsg->getSource();
    des = tempmsg->getDestination();
    printf("[domain%d:forwardMessageToSlave] Forwarding message '%s' to slave controllers. Src = %d, Des = %d.\n", getIndex(), msg->getName(), src, des);
    send(msg->dup(), "slave$o", 0);
    send(msg->dup(), "slave$o", 1);
}

void domain_controller::copyCondition(condition* cond) {
    printf("[domain%d:copyCondition] Copying network condition data.\n", getIndex());
    for (int i = 0; i < 20; ++i)
        for (int j = 0; j < 20; ++j) {
            cond->loss[i][j] = loss[i][j];
            cond->transmissionDelay[i][j] = transmissionDelay[i][j];
            cond->queuingDelay[i][j] = queuingDelay[i][j];
            cond->availableBandwidth[i][j] = availableBandwidth[i][j];
            cond->totalBandwidth[i][j] = totalBandwidth[i][j];
            cond->G[i][j] = G[i][j];
        }
}

void domain_controller::retrieveCondition(cMessage* msg) {
    printf("[domain%d:retrieveCondition] Retrieving condition from message '%s'.\n", getIndex(), msg->getName());
    condition* cond = (condition*) msg->getObject("");
    for (int i = 0; i < 20; ++i)
        for (int j = 0; j < 20; ++j) {
            if (loss[i][j] == -1)
                loss[i][j] = cond->loss[i][j];
            if (transmissionDelay[i][j] == -1)
                transmissionDelay[i][j] = cond->transmissionDelay[i][j];
            if (queuingDelay[i][j] == -1)
                queuingDelay[i][j] = cond->queuingDelay[i][j];
            if (availableBandwidth[i][j] == -1)
                availableBandwidth[i][j] = cond->availableBandwidth[i][j];
            if (totalBandwidth[i][j] == -1)
                totalBandwidth[i][j] = cond->totalBandwidth[i][j];
            if (G[i][j] == 0)
                G[i][j] = cond->G[i][j];
        }
    getIsIn();
}


void domain_controller::finish() {
    // Determine output filename based on the domain controller's index.
    const char* filename = (getIndex() == 0 ? "domain_1_Q_table.txt" : "domain_2_Q_table.txt");
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        error("Cannot open file %s for writing Q-table.", filename);
    }

    // Write the entire Q-table values.
    // Q is a 3D array of size [20][20][20].
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 20; j++) {
            for (int k = 0; k < 20; k++) {
                fprintf(fp, "Q[%d][%d][%d] = %f\n", i, j, k, Q[i][j][k]);
            }
        }
    }

    fclose(fp);
    printf("[domain%d:finish] Q-table written to %s.\n", getIndex(), filename);
}

