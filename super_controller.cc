#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include <route.h>
#include <condition.h>
#include <node.h>
#include "switch_message_m.h"

using namespace omnetpp;
class QUpdate : public cObject {
  public:
    double delta[20][20][20];
    QUpdate() { memset(delta, 0, sizeof(delta)); }
    virtual QUpdate* dup() const override { return new QUpdate(*this); }
};


class super_controller : public cSimpleModule
{
    public:
        double loss[20][20];
        double transmissionDelay[20][20];
        double queuingDelay[20][20];
        double availableBandwidth[20][20];
        double totalBandwidth[20][20];
        int src, des, nex[20][20], get;
        bool G[20][20];
        int visit[20][20];
        double Q[20][20][20];
        double epsilon = 0.1, tau0 = 100, tauT = 0.5, T = 100;
        double beta1 = 1, beta2 = 1, beta3 = 1, theta1 = 1, phi1 = 1;
        double theta2 = 0.5, phi2 = 0.5;
        double alpha = 1, gamma = 1;
        double pi = acos(-1.0);

    protected:
      virtual void forwardMessageToDomain();
      virtual void updateRouteOfDomain();
      virtual void initialize() override;
      virtual double getTau();
      virtual void handleMessage(cMessage *msg) override;
      virtual void retrieveCondition(cMessage* msg);
      virtual void finish() override;
      virtual double getReward(int state, int nextState);
      virtual void sarsa(double (& q)[20][20]);
      int makeSoftmaxPolicy(int state, double (&q)[20]);
};

Define_Module(super_controller);

double super_controller::getTau() {
    double tau = -((tau0 - tauT) * visit[src][des]) / T + tau0;
    printf("[super:getTau] visit[%d][%d] = %d, computed tau = %f.\n", src, des, visit[src][des], tau);
    return tau;
}

void printVector(int src, vector<Node*> q) {
    int len = q.size();
    printf("[super:printVector] For state %d, vector length = %d: ", src, len);
    for (int i = 0; i < len; ++i)
        printf("%d ", q[i]->idx);
    printf("\n");
}

bool cmp(Node* a, Node* b) {
    return a->val > b->val;
}

int super_controller::makeSoftmaxPolicy(int state, double (&q)[20]) {

    std::vector<Node*> candidates;
    for (int i = 0; i < 20; ++i) {
        if (G[state][i]) {
            candidates.push_back(new Node(i, q[i]));
        }
    }

    // If there are no valid actions, return an error code (-1).
    if (candidates.empty()) {
        return -1;
    }

    int len = candidates.size();
    double tau = getTau();
    double sum = 0;


    for (int i = 0; i < len; ++i) {
        candidates[i]->val = exp(candidates[i]->val / tau);
        sum += candidates[i]->val;
    }

    for (int i = 0; i < len; ++i) {
        candidates[i]->val /= sum;

    }

    for (int i = 0; i < candidates.size(); ++i) {
          std::cout << "propability of " << candidates[i]->idx  << " value: " << candidates[i]->val << std::endl;
      }



    double r = uniform(0, 1);
    for (int i = 0; i < len; ++i) {
        if (r <= candidates[i]->val) {
            int chosenAction = candidates[i]->idx;
            for (int j = 0; j < len; ++j)
                delete candidates[j];
            return chosenAction;
        }
        else {
            r -= candidates[i]->val;
        }
    }

    // Fallback: return the last candidate if none selected.
    int fallbackAction = candidates[len - 1]->idx;
    for (int j = 0; j < len; ++j)
        delete candidates[j];
    return fallbackAction;
}

double super_controller::getReward(int state, int nextState) {
    double totalQueuingDelay = 0;
    double totalTransmissionDelay = 0;
    double totalAvailableBandwidth = 0;
    int n = 0; // number of the surrounding nodes
    for (int i = 0; i < 20; ++i) {
        if (G[state][i]) {
            ++n;
            totalQueuingDelay += queuingDelay[state][i];
            totalTransmissionDelay += transmissionDelay[state][i];
            totalAvailableBandwidth += availableBandwidth[state][i];
        }
    }


    double delay = (2 * atan(transmissionDelay[state][nextState] - totalTransmissionDelay / n)) / pi;
    double queue = (2 * atan(queuingDelay[state][nextState] - totalQueuingDelay / n)) / pi;
    double los = 1 - 2 * loss[state][nextState];
    double B1 = (2 * availableBandwidth[state][nextState] / totalBandwidth[state][nextState]) -1;
    double B2 = 2 * atan(0.01 * (availableBandwidth[state][nextState] - totalAvailableBandwidth / n)) / pi;

    double reward = -3 + beta1 * (theta1 * delay + theta2 * queue) + beta2 * los + beta3 * (phi1 * B1 + phi2 * B2);
    printf("[super:getReward] For state %d to nextState %d, reward = %f.\n", state, nextState, reward);
    return reward;
}

void super_controller::sarsa(double (& q)[20][20]) {
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
    printf("[super:sarsa] Running SARSA for route from %d to %d.\n", src, des);

    while (true) {
        nex[state][des] = action;
        nextState = action;
        nextAction = makeSoftmaxPolicy(nextState, q[nextState]);
        reward = getReward(state, action);
        q[state][action] += alpha * (reward + gamma * q[nextState][nextAction] - q[state][action]);
        printf("[super:sarsa] Updated Q[%d][%d] = %f.\n", state, action, q[state][action]);
        if (nextState == des)
            break;
        action = nextAction;
        state = nextState;
    }
    printf("[super:sarsa] Completed SARSA for route from %d to %d. visit[%d][%d] = %d.\n", src, des, src, des, visit[src][des]);
}

void super_controller::initialize()
{
    for (int i = 0; i < 20; ++i)
    for (int j = 0; j < 20; ++j) {

            loss[i][j] = -1;
            transmissionDelay[i][j] = -1;
            queuingDelay[i][j] =-1;
            availableBandwidth[i][j] = -1;
            totalBandwidth[i][j] =-1;
            nex[i][j] = -1;
            visit[i][j] = 0;
            G[i][j] = 0;
    }
    get = 0;
    printf("[super:initialize] Initialized super controller. get = %d.\n", get);
}

void super_controller::updateRouteOfDomain() {

    cMessage* msg = new cMessage("update");
    Route* route = new Route();

    for (int i = 0; i < 20; ++i)
    for (int j = 0; j < 20; ++j)
    route->nex[i][j] = nex[i][j];

    msg->addObject(route);
    msg->addPar("des").setLongValue(des);
    msg->addPar("src").setLongValue(src);
    printf("[super:updateRouteOfDomain] Sending updated route plan from src %d to des %d.\n", src, des);
    send(msg->dup(), "domain$o", 0);
    send(msg->dup(), "domain$o", 1);
}

void super_controller::handleMessage(cMessage *msg)
{
    // must come from domain controller
    string name = msg->getName();
    if (name == "retrieve") {
        ++get;
        retrieveCondition(msg);
        printf("[super:handleMessage] Received 'retrieve' message. get = %d.\n", get);
        if (get == 2) {
            get = 0;
            printf("[super:handleMessage] All condition messages received. Running SARSA.\n");
            sarsa(Q[des]);
            int l = src;
            printf("[super:handleMessage] Calculated route: ");
            while (l != des) {
                printf("%d -> ", l);
                l = nex[l][des];
            }
            printf("%d\n", des);
            updateRouteOfDomain();
        }
    } else if (name == "turn in") {
        switch_message* tempmsg = check_and_cast<switch_message *>(msg);
        src = tempmsg->getSource();
        des = tempmsg->getDestination();
        printf("[super:handleMessage] Received 'turn in' message. Src = %d, Des = %d.\n", src, des);
        forwardMessageToDomain();
    }else if (name == "Federated update") {
        // Retrieve the QUpdate object from the message.
        QUpdate* updateObj = check_and_cast<QUpdate*>(msg->getObject(0));
        // For every entry in the global Q table, update:
        // Q[i][j][k] = ( Q[i][j][k] + updateObj->delta[i][j][k] )
        for (int i = 0; i < 20; i++) {
            for (int j = 0; j < 20; j++) {
                for (int k = 0; k < 20; k++) {
                    double old_value = Q[i][j][k];
                    Q[i][j][k] = (Q[i][j][k] + updateObj->delta[i][j][k]);
                    if (updateObj->delta[i][j][k] !=0 )
                    std::cout << "[super: Federated update status] Q[" << i << "][" << j << "][" << k << "]: old="
                              << old_value << ", delta=" << updateObj->delta[i][j][k]
                              << ", new=" << Q[i][j][k] << std::endl;
                }
            }
        }
        std::cout << "[super:handleMessage] Processed Federated update from domain controller." << std::endl;
        delete msg;
    }
}

void super_controller::forwardMessageToDomain()
{
    switch_message* msg = new switch_message("request");
    msg->setSource(src);
    msg->setDestination(des);
    printf("[super:forwardMessageToDomain] Forwarding 'request' to domain for route from %d to %d.\n", src, des);
    send(msg->dup(), "domain$o", 0);
    send(msg->dup(), "domain$o", 1);
}

void super_controller::retrieveCondition(cMessage* msg) {
    printf("[super:retrieveCondition] Retrieving condition from message '%s'.\n", msg->getName());
    condition* cond = (condition*) msg->getObject("");
    for (int i = 0; i < 20; ++i) {
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
    }
}

void super_controller::finish() {
    std::ofstream outFile("Q_table.txt");
    if (outFile.is_open()) {
        for (int i = 0; i < 20; ++i) {
            for (int j = 0; j < 20; ++j) {
                for (int k = 0; k < 20; ++k) { 
                    if (Q[i][j][k] < 1)
                    outFile << "Q[" << i << "][" << j << "][" << k << "] = " << Q[i][j][k] << "\n";
                }
            }
        }
        outFile.close();
        printf("[super:finish] Q table written to Q_table.txt.\n");
    } else {
        printf("[super:finish] Unable to open file Q_table.txt for writing.\n");
    }
}
