#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <condition.h>
#include "switch_message_m.h"

using namespace omnetpp;
using namespace std;

class slave_controller : public cSimpleModule
{
    private:
        double loss[20][20], transmissionDelay[20][20], queuingDelay[20][20];
        double availableBandwidth[20][20], totalBandwidth[20][20];
        int nex[20][20][20];
        bool G[20][20];

    protected:
      virtual void initialize() override;
      virtual void handleMessage(cMessage *msg) override;
      virtual void copyCondition(condition* cond);
};

Define_Module(slave_controller);

void slave_controller::initialize()
{
    printf("[slave%d:initialize] Initializing slave controller.\n", getIndex());
    memset(G, 0, sizeof(G));
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            loss[i][j] = transmissionDelay[i][j] = -1;
            queuingDelay[i][j] = availableBandwidth[i][j] = -1;
            totalBandwidth[i][j] = -1;
        }
    }
}

void slave_controller::handleMessage(cMessage *msg)
{
    printf("[slave%d:handleMessage] Received message '%s' from '%s'.\n", getIndex(), msg->getName(), msg->getSenderModule()->getName());
    string from = msg->getSenderModule()->getName();

    if (from == "switches") {
        switch_message* tempmsg = check_and_cast<switch_message *>(msg);
        int idx = tempmsg->getSource();
        condition* cond = (condition*) tempmsg->getObject("");

        printf("[slave%d:handleMessage] Processing condition data from switches message with source %d.\n", getIndex(), idx);
        for (int i = 0; i < 20; ++i) {
            if (loss[idx][i] <= 0) loss[idx][i] = cond->loss[idx][i];
            if (transmissionDelay[idx][i] <= 0) transmissionDelay[idx][i] = cond->transmissionDelay[idx][i];
            if (queuingDelay[idx][i] <= 0) queuingDelay[idx][i] = cond->queuingDelay[idx][i];
            if (availableBandwidth[idx][i] <= 0) availableBandwidth[idx][i] = cond->availableBandwidth[idx][i];
            if (totalBandwidth[idx][i] <= 0) totalBandwidth[idx][i] = cond->totalBandwidth[idx][i];
            if (G[idx][i] == 0) G[idx][i] = cond->G[idx][i];
        }

        // Debug output for a specific condition, if needed.


    } else if (from == "domain") {
        printf("[slave%d:handleMessage] Received message '%s' from domain. Preparing to send condition info back.\n", getIndex(), msg->getName());
        condition* cond = new condition();
        copyCondition(cond);
        cMessage* msg_ = new cMessage();
        msg_->addObject(cond);
        printf("[slave%d:handleMessage] Sending condition information back to domain.\n", getIndex());
        send(msg_->dup(), "domain$o");
    }
}

void slave_controller::copyCondition(condition* cond) {
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            cond->loss[i][j] = loss[i][j];
            cond->transmissionDelay[i][j] = transmissionDelay[i][j];
            cond->queuingDelay[i][j] = queuingDelay[i][j];
            cond->availableBandwidth[i][j] = availableBandwidth[i][j];
            cond->totalBandwidth[i][j] = totalBandwidth[i][j];
            cond->G[i][j] = G[i][j];
        }
    }
}
