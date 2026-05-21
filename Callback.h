#include <stdint.h> // uint64_t

#ifndef _LP_CALLBACK_H
#define _LP_CALLBACK_H

namespace LPDDRSim {

template <typename ReturnT, typename Param1T, typename Param2T, typename Param3T, typename Param4T, typename Param5T>

class CallbackBase {
public:
    virtual ~CallbackBase() = 0;
    virtual ReturnT operator()(Param1T, Param2T, Param3T, Param4T, Param5T) = 0;
};

template <typename Return, typename Param1T, typename Param2T, typename Param3T, typename Param4T, typename Param5T>
LPDDRSim::CallbackBase<Return,Param1T,Param2T,Param3T,Param4T,Param5T>::~CallbackBase() {}

template <typename ConsumerT, typename ReturnT, typename Param1T, typename Param2T,
         typename Param3T, typename Param4T, typename Param5T>

class Callback: public CallbackBase<ReturnT,Param1T,Param2T,Param3T,Param4T,Param5T> {
private:
    typedef ReturnT (ConsumerT::*PtrMember)(Param1T,Param2T,Param3T,Param4T,Param5T);

public:
    Callback(ConsumerT* const object, PtrMember member) :
        object(object), member(member) {
    }

    Callback(const Callback<ConsumerT,ReturnT,Param1T,Param2T,Param3T,Param4T,Param5T>& e) :
        object(e.object), member(e.member) {
    }

    ReturnT operator()(Param1T param1, Param2T param2, Param3T param3, Param4T param4, Param5T param5) {
        return (const_cast<ConsumerT*>(object)->*member)
               (param1,param2,param3,param4,param5);
    }

private:

    ConsumerT* const object;
    const PtrMember  member;
};

typedef CallbackBase <bool, unsigned, uint64_t, double, double, double> TransactionCompleteCB;
} // namespace DDRSim

#endif