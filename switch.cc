#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <string>
#include <condition.h>
#include "switch_message_m.h"

using namespace omnetpp;
using namespace std;

class sdn_switch : public cSimpleModule
{
    private:
      simsignal_t arrivalSignal;
      simtime_t timeout;  // timeout
      cMessage *timeoutEvent;  // holds pointer to the timeout self-message
      switch_message *msg_;
      double loss[20][20], transmissionDelay[20][20], queuingDelay[20][20];
      double availableBandwidth[20][20], totalBandwidth[20][20];
      int nex[20]; // nex[des]: the next hop if the destination of the packet is 'des'
      int idx;
      bool G[20][20];

    protected:
      virtual switch_message *generateMessage(char *a);
      virtual void forwardMessageToSwitch(switch_message *msg, int to);
      virtual void forwardMessageToDomain(switch_message *msg);
      virtual void forwardMessageToSlave(switch_message *msg);
      virtual void initialize() override;
      virtual void handleMessage(cMessage *msg) override;
      virtual void recordInformation(switch_message *msg);
      virtual switch_message* initializationMessage();
};

Define_Module(sdn_switch);

switch_message* sdn_switch::initializationMessage() {
    printf("[switch%d:initializationMessage] Initializing switch message.\n", getIndex());
    switch_message* msg = new switch_message("ini");
    msg->setSource(getIndex());
    condition* cond = new condition();

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

    msg->addObject(cond);
    return msg;
}

void sdn_switch::initialize()
{
    printf("[switch%d:initialize] Initializing switch.\n", getIndex());
    timeout = 1.0;
    timeoutEvent = new cMessage("timeoutEvent");

    arrivalSignal = registerSignal("arrival");
    idx = getIndex();

    for (int i = 0; i < 20; ++i)
        for (int j = 0; j < 20; ++j) {
            loss[i][j] = transmissionDelay[i][j] = -1;
            queuingDelay[i][j] = availableBandwidth[i][j] = -1;
            totalBandwidth[i][j] = nex[i] = -1;
        }

    memset(G, 0, sizeof(G));
    printf("[switch%d:initialize] Connected nodes: ", idx);
    for (cModule::GateIterator i(this); !i.end(); i++)
    {
         cGate *gate = i();
         std::string gateStr = gate->getName();
         if (gateStr == "gate$o")
         {
             int to = gate->getPathEndGate()->getOwnerModule()->getIndex();
             G[idx][to] = 1;
             printf("%d ", to);
             loss[idx][to] = par("loss");
             transmissionDelay[idx][to] = par("transmissionDelay");
             queuingDelay[idx][to] = par("queuingDelay");
             availableBandwidth[idx][to] = par("availableBandwidth");
             totalBandwidth[idx][to] = par("totalBandwidth");
             while (totalBandwidth[idx][to] < availableBandwidth[idx][to])
                 totalBandwidth[idx][to] = par("totalBandwidth");
         }
    }
    printf("\n");

    scheduleAt(0.0, initializationMessage()->dup());

    string name = getName();
    // Module 0 sends the first message
    if (getIndex() == 0 && name == "switches") {
        printf("[switch%d:initialize] About to send the initial message.\n", getIndex());
        msg_ = generateMessage("msg");
        scheduleAt(1.0, msg_);
    }
}

void sdn_switch::handleMessage(cMessage *msg)
{
    // Display the switch index and the type of message being handled.
    printf("[switch%d:%s] Handling message from %s.\n", getIndex(), msg->getName(), msg->getSenderModule()->getName());
    string from = msg->getSenderModule()->getName();
    string name = msg->getName();

    if (name == "ini") {
        send(msg->dup(), "slave");
        return ;
    }

    if (from == "switches") {
        if (name == "msg") {
            switch_message* tempmsg = check_and_cast<switch_message *>(msg);
            tempmsg->setSource(getIndex());
            recordInformation(tempmsg);
            forwardMessageToSlave(tempmsg);
            printf("[switch%d:handleMessage] Forwarding message '%s' to slave.\n", getIndex(), tempmsg->getName());

            if (tempmsg->getDestination() == getIndex()) {
                printf("[switch%d:handleMessage] Message '%s' arrived at destination. Generating new one.\n", getIndex(), tempmsg->getName());
                int hopcount = tempmsg->getHopCount();
                emit(arrivalSignal, hopcount);
                msg_ = generateMessage("msg");
                int des = msg_->getDestination();

                bubble((std::string("Message arived to dest ! \n [switch") + std::to_string(getIndex()) + ":genMessage] Generated new message from " + std::to_string(msg_->getSource()) + " to " + std::to_string(msg_->getDestination())).c_str());
                if (nex[des] != -1)
                    forwardMessageToSwitch(msg_->dup(), nex[des]);
                else
                    forwardMessageToDomain(msg_);
            } else {
                int des = tempmsg->getDestination();
                if (nex[des] != -1)
                    forwardMessageToSwitch(tempmsg->dup(), nex[des]);
                else
                    forwardMessageToDomain(msg_);
            }
        }
    } else if (from == "domain") {
        int des = (int) msg->par("des").longValue();
        int hop = (int) msg->par("hop").longValue();

        if (name == "update") {
            nex[des] = hop;
        } else if (name == "send") {
            printf("[switch%d:handleMessage] Received 'send' request from domain for message '%s'. Forwarding message.\n", getIndex(), msg->getName());
            forwardMessageToSwitch(msg_, nex[des]);
        }
    }
}

switch_message *sdn_switch::generateMessage(char *a)
{
    int src = getIndex();
    int n = getVectorSize();
    int dest = intuniform(0, n-2);
    if (dest >= src)
        dest++;

    switch_message *msg = new switch_message(a);
    msg->setDestination(dest);
    msg->setSource(src);

    printf("[switch%d:generateMessage] Generated new message '%s' from %d to %d.\n", getIndex(), a, src, dest);
    return msg;
}

void sdn_switch::forwardMessageToSwitch(switch_message *msg, int to)
{
    printf("[switch%d:forwardMessageToSwitch] Forwarding message '%s' to switch %d.\n", getIndex(), msg->getName(), to);
    msg->setHopCount(msg->getHopCount() + 1);

    for (cModule::GateIterator i(this); !i.end(); i++)
    {
         cGate *gate = i();
         std::string gateStr = gate->getName();
         if (gateStr == "gate$o" && gate->getPathEndGate()->getOwnerModule()->getIndex() == to)
         {
             send(msg->dup(), "gate$o", gate->getIndex());
         }
    }
}

void sdn_switch::forwardMessageToDomain(switch_message *msg) {
    printf("[switch%d:forwardMessageToDomain] Forwarding message '%s' to domain.\n", getIndex(), msg->getName());
    send(msg->dup(), "domain$o");
}

void sdn_switch::forwardMessageToSlave(switch_message *msg) {
    printf("[switch%d:forwardMessageToSlave] Forwarding message '%s' to slave.\n", getIndex(), msg->getName());
    send(msg->dup(), "slave");
}

void sdn_switch::recordInformation(switch_message *msg) {
    condition* cond = new condition();

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

    msg->addObject(cond);
}
