//
// Generated file, do not edit! Created by nedtool 5.6 from switch_message.msg.
//

#ifndef __SWITCH_MESSAGE_M_H
#define __SWITCH_MESSAGE_M_H

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#include <omnetpp.h>

// nedtool version check


/**
 * Class generated from <tt>switch_message.msg:2</tt> by nedtool.
 * <pre>
 * message switch_message
 * {
 *     int source;
 *     int destination;
 *     double loss[20];
 *     double transmissionDelay[20];
 *     double queuingDelay[20];
 *     double availableBandwidth[20];
 *     double totalBandwidth[20];
 *     int hopCount = 0;
 * }
 * </pre>
 */
class switch_message : public ::omnetpp::cMessage
{
  protected:
    int source;
    int destination;
    double loss[20];
    double transmissionDelay[20];
    double queuingDelay[20];
    double availableBandwidth[20];
    double totalBandwidth[20];
    int hopCount;

  private:
    void copy(const switch_message& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const switch_message&);

  public:
    switch_message(const char *name=nullptr, short kind=0);
    switch_message(const switch_message& other);
    virtual ~switch_message();
    switch_message& operator=(const switch_message& other);
    virtual switch_message *dup() const override {return new switch_message(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual int getSource() const;
    virtual void setSource(int source);
    virtual int getDestination() const;
    virtual void setDestination(int destination);
    virtual unsigned int getLossArraySize() const;
    virtual double getLoss(unsigned int k) const;
    virtual void setLoss(unsigned int k, double loss);
    virtual unsigned int getTransmissionDelayArraySize() const;
    virtual double getTransmissionDelay(unsigned int k) const;
    virtual void setTransmissionDelay(unsigned int k, double transmissionDelay);
    virtual unsigned int getQueuingDelayArraySize() const;
    virtual double getQueuingDelay(unsigned int k) const;
    virtual void setQueuingDelay(unsigned int k, double queuingDelay);
    virtual unsigned int getAvailableBandwidthArraySize() const;
    virtual double getAvailableBandwidth(unsigned int k) const;
    virtual void setAvailableBandwidth(unsigned int k, double availableBandwidth);
    virtual unsigned int getTotalBandwidthArraySize() const;
    virtual double getTotalBandwidth(unsigned int k) const;
    virtual void setTotalBandwidth(unsigned int k, double totalBandwidth);
    virtual int getHopCount() const;
    virtual void setHopCount(int hopCount);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const switch_message& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, switch_message& obj) {obj.parsimUnpack(b);}


#endif // ifndef __SWITCH_MESSAGE_M_H

